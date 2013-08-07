ncollide_doc_path=doc
ncollide_rc=src/ncollide.rc
ncollide_lib_path=lib
nalgebra_lib_path=./nalgebra/lib

all:
	mkdir -p $(ncollide_lib_path)
	rust build $(ncollide_rc) -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3

deps:
	make -C nalgebra

test:
	mkdir -p $(ncollide_lib_path)
	rustc -L$(nalgebra_lib_path) --test $(ncollide_rc) -o test~ && ./test~
	rm test~

bench:
	mkdir -p $(ncollide_lib_path)
	rustc -L$(nalgebra_lib_path) --test $(ncollide_rc) --opt-level 3 -o bench~ && ./bench~ --bench
	rm bench~

doc:
	rust doc $(ncollide_rc) --output-dir $(ncollide_doc_path)

.PHONY:doc
.PHONY:test
