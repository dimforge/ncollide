tmp=_git_distcheck

all:
	cargo build --release

test:
	cd build/ncollide2df32; cargo test
	cd build/ncollide3df32; cargo test
	cd build/ncollide4df32; cargo test
	cd build/ncollide2df64; cargo test
	cd build/ncollide3df64; cargo test
	cd build/ncollide4df64; cargo test

distcheck:
	rm -rf $(tmp)
	git clone . $(tmp)
	make -C $(tmp)
	rm -rf $(tmp)

doc:
	cargo doc

clean:
	cargo clean
