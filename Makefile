ncollide_doc_path=doc
ncollide_rc=src/ncollide.rc
ncollide_lib_path=lib
nalgebra_lib_path=./nalgebra/lib

all:
	rust build $(ncollide_rc) -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path)

deps:
	make -C nalgebra

test:
	rustc -L$(nalgebra_lib_path) --test $(ncollide_rc) -o test~ && ./test~
	rm test~

doc:
	rust doc $(ncollide_rc) --output-dir $(ncollide_doc_path)

.PHONY:doc
.PHONY:test
