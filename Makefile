tmp=_git_distcheck
ncollide_doc_path=doc
ncollide_rs=src/ncollide.rs
ncollide_lib_path=lib
nalgebra_lib_path=./nalgebra/lib

all:
	mkdir -p $(ncollide_lib_path)
	rustc $(ncollide_rs) -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3

deps:
	make -C nalgebra

test:
	mkdir -p $(ncollide_lib_path)
	rustc -L$(nalgebra_lib_path) --test $(ncollide_rs) --opt-level 3 --link-args -lm -o test~ && ./test~
	rm test~

bench:
	mkdir -p $(ncollide_lib_path)
	rustc -L$(nalgebra_lib_path) --test $(ncollide_rs) --opt-level 3 --link-args -lm -o bench~ && ./bench~ --bench
	rm bench~

distcheck:
	rm -rf $(tmp)
	git clone --recursive . $(tmp)
	make -C $(tmp) deps
	make -C $(tmp)
	make -C $(tmp) test
	make -C $(tmp) bench
	rm -rf $(tmp)

doc:
	mkdir -p $(ncollide_doc_path)
	rustdoc -L$(nalgebra_lib_path) src/ncollide.rs

.PHONY:doc
.PHONY:test
.PHONY:bench
