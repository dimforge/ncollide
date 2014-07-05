tmp=_git_distcheck
ncollide_doc_path=doc
ncollide_rs=src/lib.rs
ncollide_lib_path=lib
nalgebra_lib_path=./nalgebra/lib

all: 3df32 2df32 4df32 2df64 3df64 4df64
	mkdir -p $(ncollide_lib_path)

2df32:
	mkdir -p $(ncollide_lib_path)
	rustc src/ncollide2df32.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim2
3df32:
	mkdir -p $(ncollide_lib_path)
	rustc src/ncollide3df32.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim3
4df32:
	mkdir -p $(ncollide_lib_path)
	rustc src/ncollide4df32.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim4
2df64:
	mkdir -p $(ncollide_lib_path)
	rustc src/ncollide2df64.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim2
3df64:
	mkdir -p $(ncollide_lib_path)
	rustc src/ncollide3df64.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim3
4df64:
	mkdir -p $(ncollide_lib_path)
	rustc src/ncollide4df64.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim4

deps:
	make -C nalgebra

test:
	mkdir -p $(ncollide_lib_path)
	rustc -L$(nalgebra_lib_path) --test src/ncollide3df32.rs --opt-level 3 --cfg dim3 -o test~ && ./test~
	rustc -L$(nalgebra_lib_path) --test src/ncollide2df32.rs --opt-level 3 --cfg dim2 -o test~ && ./test~
	rustc -L$(nalgebra_lib_path) --test src/ncollide3df64.rs --opt-level 3 --cfg dim3 -o test~ && ./test~
	rustc -L$(nalgebra_lib_path) --test src/ncollide2df64.rs --opt-level 3 --cfg dim2 -o test~ && ./test~

bench:
	mkdir -p $(ncollide_lib_path)
	rustc -L$(nalgebra_lib_path) --test src/ncollide3df64.rs --opt-level 3 --cfg dim3 -o bench~ && ./bench~ --bench
	rustc -L$(nalgebra_lib_path) --test src/ncollide2df32.rs --opt-level 3 --cfg dim2 -o bench~ && ./bench~ --bench
	rustc -L$(nalgebra_lib_path) --test src/ncollide3df32.rs --opt-level 3 --cfg dim3 -o bench~ && ./bench~ --bench
	rustc -L$(nalgebra_lib_path) --test src/ncollide2df64.rs --opt-level 3 --cfg dim2 -o bench~ && ./bench~ --bench
	rm bench~

distcheck:
	rm -rf $(tmp)
	git clone --recursive . $(tmp)
	make -C $(tmp) deps
	make -C $(tmp)
	make -C $(tmp) test
	make -C $(tmp) bench
	make -C $(tmp) doc
	rm -rf $(tmp)

doc:
	mkdir -p $(ncollide_doc_path)
	rustdoc -L$(nalgebra_lib_path) --cfg dim2 src/ncollide2df64.rs
	rustdoc -L$(nalgebra_lib_path) --cfg dim3 src/ncollide3df64.rs
	rustdoc -L$(nalgebra_lib_path) --cfg dim4 src/ncollide4df64.rs
	rustdoc -L$(nalgebra_lib_path) --cfg dim2 src/ncollide2df32.rs
	rustdoc -L$(nalgebra_lib_path) --cfg dim3 src/ncollide3df32.rs
	rustdoc -L$(nalgebra_lib_path) --cfg dim4 src/ncollide4df32.rs

.PHONY:doc
.PHONY:test
.PHONY:bench
