tmp=_git_distcheck
ncollide_doc_path=doc
ncollide_rs=src/lib.rs
ncollide_lib_path=lib
nalgebra_lib_path=./nalgebra/lib

all:
	mkdir -p $(ncollide_lib_path)
	rustc src/lib2df64.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim2 --cfg f64
	rustc src/lib3df64.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim3 --cfg f64
	rustc src/lib4df64.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim4 --cfg f64
	rustc src/lib2df32.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim2 --cfg f32
	rustc src/lib3df32.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim3 --cfg f32
	rustc src/lib4df32.rs -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --cfg dim4 --cfg f32

deps:
	make -C nalgebra

test:
	mkdir -p $(ncollide_lib_path)
	rustc -L$(nalgebra_lib_path) --test src/lib3df64.rs --opt-level 3 --cfg dim3 --cfg f64 -o test~ && ./test~
	rustc -L$(nalgebra_lib_path) --test src/lib2df32.rs --opt-level 3 --cfg dim2 --cfg f32 -o test~ && ./test~
	rustc -L$(nalgebra_lib_path) --test src/lib3df32.rs --opt-level 3 --cfg dim3 --cfg f32 -o test~ && ./test~
	rustc -L$(nalgebra_lib_path) --test src/lib2df64.rs --opt-level 3 --cfg dim2 --cfg f64 -o test~ && ./test~

bench:
	mkdir -p $(ncollide_lib_path)
	rustc -L$(nalgebra_lib_path) --test src/lib3df64.rs --opt-level 3 --cfg dim3 --cfg f64 -o bench~ && ./bench~ --bench
	rustc -L$(nalgebra_lib_path) --test src/lib2df32.rs --opt-level 3 --cfg dim2 --cfg f32 -o bench~ && ./bench~ --bench
	rustc -L$(nalgebra_lib_path) --test src/lib3df32.rs --opt-level 3 --cfg dim3 --cfg f32 -o bench~ && ./bench~ --bench
	rustc -L$(nalgebra_lib_path) --test src/lib2df64.rs --opt-level 3 --cfg dim2 --cfg f64 -o bench~ && ./bench~ --bench
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
	rustdoc -L$(nalgebra_lib_path) --cfg dim2 --cfg f64 src/lib2df64.rs
	rustdoc -L$(nalgebra_lib_path) --cfg dim3 --cfg f64 src/lib3df64.rs
	rustdoc -L$(nalgebra_lib_path) --cfg dim4 --cfg f64 src/lib4df64.rs
	rustdoc -L$(nalgebra_lib_path) --cfg dim2 --cfg f32 src/lib2df32.rs
	rustdoc -L$(nalgebra_lib_path) --cfg dim3 --cfg f32 src/lib3df32.rs
	rustdoc -L$(nalgebra_lib_path) --cfg dim4 --cfg f32 src/lib4df32.rs

.PHONY:doc
.PHONY:test
.PHONY:bench
