tmp=_git_distcheck

all:
	# cargo build  --no-default-features
	cargo build

test:
	RUST_BACKTRACE=1 cargo test

distcheck:
	rm -rf $(tmp)
	git clone . $(tmp)
	make -C $(tmp)
	make test -C $(tmp)
	make bench -C $(tmp)
	rm -rf $(tmp)

doc:
	cargo doc --no-deps

bench:
	cargo bench

clean:
	cargo clean
