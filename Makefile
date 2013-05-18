ncollide_rc=src/ncollide.rc
ncollide_lib_path=lib
nalgebra_lib_path=../nalgebra/lib

all:
	rust build $(ncollide_rc) -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path)

test:
	rustc -L$(nalgebra_lib_path) --test $(ncollide_rc) -o test~ && ./test~
	rm test~

doc:
	rust test $(ncollide_rc)

.PHONY:doc, test
