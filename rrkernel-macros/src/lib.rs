//! Attribute macros for rrkernel.
//!
//! One macro, [`macro@rrkernel`], and everything it removes from an application: the
//! kernel configuration call site, the "main is task 0" plumbing, the exit path, the
//! vector-table wiring, the fault handlers and the panic handler.

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn};

/// Turn a `cortex-m-rt` `main` into the kernel's **task 0**.
///
/// ```ignore
/// #![no_std]
/// #![no_main]
///
/// use rrkernel::{scheduler, thread, Slice};
///
/// const CORE_HZ: u32 = 8_000_000;
///
/// #[rrkernel]
/// #[cortex_m_rt::entry]           // keep the ecosystem's entry point
/// fn main() -> ! {
///     // optional, one line: where a panic or a fault should be reported
///     rrkernel::log_with(|a| { /* e.g. rtt_target::rprintln!("{}", a) */ });
///
///     // the kernel, configured once. No `Result` to ignore: it either starts or says
///     // why through the log above and then parks.
///     rrkernel::configure(CORE_HZ, Slice::Millis(1), 1024);
///
///     thread::spawn(worker);
///     loop { scheduler::sleep_ticks(1000); }
/// }
///
/// fn worker() { /* ... */ }
/// ```
///
/// What it expands to:
///
/// * the function itself, with its attributes preserved, its signature forced to `-> !`
///   (the body is followed by the exit path, so it cannot return);
/// * `rrkernel::app_support::exit_main()` after the body: `main` *is* task 0, so returning
///   unlinks it and switches away for good, exactly like any other task;
/// * a strong `HardFault`, a strong `DefaultHandler` and a `#[panic_handler]` that report
///   through the `log_with` sink before parking — so a fault can never look like a hang,
///   and the application needs no `panic-halt`.
///
/// Apply it once per program, above `#[cortex_m_rt::entry]`.
#[proc_macro_attribute]
pub fn rrkernel(attr: TokenStream, item: TokenStream) -> TokenStream {
    let mut func = parse_macro_input!(item as ItemFn);

    // The generated body ends with `exit_main()`, so the signature must not return. This
    // is also why the application does not have to write `-> !` itself.
    func.sig.output = syn::parse_quote!(-> !);

    // Re-emit only what belongs to an entry point: its attributes (notably
    // `#[cortex_m_rt::entry]`, which must survive for the vector table), visibility and
    // name. Parameters and generics are dropped; an entry point has none.
    let attrs = func.attrs.clone();
    let vis = func.vis.clone();
    let name = func.sig.ident.clone();
    let body = func.block.clone();

    // `#[rrkernel(log = rtt)]` wires RTT — terminal, guard and sink — so an application
    // writes none of that boilerplate. Anything else, including no argument at all, leaves
    // the kernel's built-in RAM buffer as the sink, which needs no setup whatsoever.
    let log_setup = match log_choice(attr).as_deref() {
        Some("rtt") => quote! {
            // RTT over the debug probe. The terminal is initialized here, and the flag
            // keeps the sink from being called before it exists.
            static RTT_UP: core::sync::atomic::AtomicBool =
                core::sync::atomic::AtomicBool::new(false);
            rtt_target::rtt_init_print!();
            RTT_UP.store(true, core::sync::atomic::Ordering::Relaxed);
            rrkernel::log_with(|a| {
                if RTT_UP.load(core::sync::atomic::Ordering::Relaxed) {
                    let _ = rtt_target::rprintln!("{}", a);
                }
            });
        },
        _ => quote! {},
    };

    quote! {
        #(#attrs)*
        #vis fn #name() -> ! {
            #log_setup
            // Mask interrupts once at startup, and (not incidentally) reference the
            // `cortex-m` crate: its `critical-section` implementation is what `rtt-target`
            // and most of the ecosystem link against, and a crate that nothing references
            // is a crate the linker never pulls in. Without this the link fails with
            // `undefined symbol: _critical_section_1_0_acquire`, which says nothing about
            // the real cause. A cast of the item is not enough — it is generic, so it needs
            // annotations — hence the call.
            let _ = cortex_m::interrupt::free(|_cs| ());
            #body
            // `main` IS task 0: its body has run to completion, so unlink it and switch
            // away for good — exactly like any other task that returns.
            rrkernel::app_support::exit_main()
        }

        #[no_mangle]
        pub extern "C" fn HardFault() -> ! {
            rrkernel::app_support::hard_fault()
        }

        #[no_mangle]
        pub extern "C" fn DefaultHandler() -> ! {
            rrkernel::app_support::unexpected_exception()
        }

        #[panic_handler]
        fn __rrkernel_panic(info: &core::panic::PanicInfo) -> ! {
            rrkernel::app_support::panic(info)
        }
    }
    .into()
}

/// Read `log = <name>` out of the attribute arguments, e.g. `#[rrkernel(log = rtt)]`.
///
/// Kept deliberately small: the only recognised value today is `rtt`. Anything else — no
/// argument, or an unknown word — means "use the kernel's built-in default sink", which
/// needs no setup and never fails to compile.
fn log_choice(attr: TokenStream) -> Option<String> {
    if attr.is_empty() {
        return None;
    }
    use syn::parse::Parser;
    let metas = syn::punctuated::Punctuated::<syn::Meta, syn::Token![,]>::parse_terminated
        .parse(attr)
        .ok()?;
    for m in metas {
        if let syn::Meta::NameValue(nv) = m {
            if nv.path.is_ident("log") {
                if let syn::Expr::Path(p) = nv.value {
                    return p.path.get_ident().map(|i| i.to_string());
                }
            }
        }
    }
    None
}
