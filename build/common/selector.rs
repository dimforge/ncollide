use syntax::ast::{MetaItem, Item};
use syntax::ext::base::{ExtCtxt, ItemModifier};
use syntax::codemap::Span;
use syntax::parse;
use syntax::parse::attr::ParserAttr;
use syntax::ptr::P;

pub struct ExpandHidden;

impl ExpandHidden {
    pub fn new() -> ExpandHidden {
        ExpandHidden
    }
}

impl ItemModifier for ExpandHidden {
    fn expand(&self, ecx: &mut ExtCtxt, span: Span, _: &MetaItem, item: P<Item>) -> P<Item> {
        let filename = ecx.parse_sess.span_diagnostic.cm.span_to_filename(span);
        let dummy_cfg = "#[cfg(_a_cfg_that_you_should_never_pass_to_rustc_because_it_breaks_things_)]";
        let mut dummy_attr_generator = parse::new_parser_from_source_str(ecx.parse_sess(), ecx.cfg(),
        filename,
        dummy_cfg.to_string());
        let attrs = dummy_attr_generator.parse_outer_attributes();

        let mut new_item = item.deref().clone();
        new_item.attrs.extend(attrs.into_iter());

        P(new_item)
    }
}

pub struct ExpandId;

impl ExpandId {
    pub fn new() -> ExpandId {
        ExpandId
    }
}

impl ItemModifier for ExpandId {
    fn expand(&self, _: &mut ExtCtxt, _: Span, _: &MetaItem, item: P<Item>) -> P<Item> {
        item
    }
}
