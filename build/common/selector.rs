use std::gc::{GC, Gc};
use syntax::ast;
use syntax::ext::base;
use syntax::codemap;
use syntax::parse;
use syntax::parse::attr::ParserAttr;


pub fn expand_hidden(cx: &mut base::ExtCtxt, _: codemap::Span,
                     _: Gc<ast::MetaItem>, input_item: Gc<ast::Item>)
    -> Gc<ast::Item>
{
    let dummy_cfg = "#[cfg(_a_cfg_that_you_should_never_pass_to_rustc_because_it_breaks_things_)]";
    let mut dummy_attr_generator = parse::new_parser_from_source_str(cx.parse_sess(), cx.cfg(),
                                                                     "dummy".to_string(),
                                                                     dummy_cfg.to_string());
    let attrs = dummy_attr_generator.parse_outer_attributes();

    let mut new_input_item = input_item.deref().clone();
    new_input_item.attrs.push_all_move(attrs);

    box(GC) new_input_item
}

pub fn expand_id(_: &mut base::ExtCtxt, _: codemap::Span,
                 _: Gc<ast::MetaItem>, input_item: Gc<ast::Item>)
                 -> Gc<ast::Item>
{
    input_item
}
