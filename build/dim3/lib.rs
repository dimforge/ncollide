#![crate_type = "dylib"]
#![feature(plugin_registrar)]
#![feature(quote)]

extern crate rustc;
extern crate syntax;

use rustc::plugin::Registry;
use syntax::parse::token;
use syntax::ext::base;
use selector::{ExpandId, ExpandHidden};

#[path="../common/selector.rs"]
mod selector;

#[plugin_registrar]
#[doc(hidden)]
pub fn plugin_registrar(reg: &mut Registry) {
    reg.register_syntax_extension(token::intern("dim2"),
        base::Modifier(box ExpandHidden::new()));
    reg.register_syntax_extension(token::intern("dim3"),
        base::Modifier(box ExpandId::new()));
    reg.register_syntax_extension(token::intern("dim4"),
        base::Modifier(box ExpandHidden::new()));
    reg.register_syntax_extension(token::intern("not_dim2"),
        base::Modifier(box ExpandId::new()));
    reg.register_syntax_extension(token::intern("not_dim3"),
        base::Modifier(box ExpandHidden::new()));
    reg.register_syntax_extension(token::intern("not_dim4"),
        base::Modifier(box ExpandId::new()));
}
