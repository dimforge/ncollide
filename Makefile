tmp=_git_distcheck
ncollide_doc_path=doc
ncollide_rs=src/lib.rs
ncollide_lib_path=lib
nalgebra_lib_path=./nalgebra/lib
compile=rustc -L$(nalgebra_lib_path) --out-dir $(ncollide_lib_path) --opt-level 3 --crate-type rlib --crate-type dylib

all:
	cargo build -u --release

test:
	cd build/ncollide2df32; cargo test
	cd build/ncollide3df32; cargo test
	cd build/ncollide4df32; cargo test
	cd build/ncollide2df64; cargo test
	cd build/ncollide3df64; cargo test
	cd build/ncollide4df64; cargo test

bench:
	mkdir -p $(ncollide_lib_path)
	rustc -L$(nalgebra_lib_path) --test src/ncollide3df64.rs --opt-level 3 --cfg dim3 -o bench~ && ./bench~ --bench
	rustc -L$(nalgebra_lib_path) --test src/ncollide2df32.rs --opt-level 3 --cfg dim2 -o bench~ && ./bench~ --bench
	rustc -L$(nalgebra_lib_path) --test src/ncollide3df32.rs --opt-level 3 --cfg dim3 -o bench~ && ./bench~ --bench
	rustc -L$(nalgebra_lib_path) --test src/ncollide2df64.rs --opt-level 3 --cfg dim2 -o bench~ && ./bench~ --bench
	rm bench~

distcheck:
	rm -rf $(tmp)
	git clone . $(tmp)
	make -C $(tmp)
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
