tmp=_git_distcheck

all:
	# cargo build  --no-default-features
	cargo build

test:
	cargo test

distcheck:
	rm -rf $(tmp)
	git clone . $(tmp)
	make -C $(tmp)
	make test -C $(tmp)
	rm -rf $(tmp)

doc:
	cargo doc

bench:
	cargo bench

clean:
	cargo clean
