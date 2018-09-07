use partitioning::PartitioningStructure;

pub enum VisitStatus {
    Continue,
    Stop,
    ExitEarly,
}

pub trait Visitor<T, BV> {
    fn visit(&mut self, bv: &BV, data: Option<&T>) -> VisitStatus;
}

pub fn visit<P, T, BV>(part: &P,
                       visitor: &mut impl Visitor<T, BV>,
                       stack: &mut Vec<P::Node>)
    where P: PartitioningStructure<T, BV> {
    if let Some(root) = part.root() {
        stack.push(root);

        while let Some(node) = stack.pop() {
            let content = part.content(node);

            match visitor.visit(content.0, content.1) {
                VisitStatus::Continue => {
                    for i in 0..part.num_children(node) {
                        stack.push(part.child(i, node))
                    }
                }
                VisitStatus::ExitEarly => return,
                VisitStatus::Stop => {}
            }
        }
    }
}