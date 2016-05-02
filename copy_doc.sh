#! /bin/sh

out_dir=./docs/rustdoc
ncollide_dir=../ncollide

echo "Generating the documentation..."
cd $ncollide_dir; make doc
cd -
rm -rf docs/rustdoc
cp -r $ncollide_dir/target/doc $out_dir

for crate in ncollide_geom ncollide_pipeline ncollide_utils ncollide_math ncollide_procedural ncollide_transformation
do
    cd $ncollide_dir/$crate; cargo doc --no-deps
    cd -
    cp -r $ncollide_dir/$crate/target/doc/$crate $out_dir/.
    cp -r $ncollide_dir/$crate/target/doc/src/$crate $out_dir/src/.
    cp -r $ncollide_dir/$crate/target/doc/implementors/$crate $out_dir/src/.
    # cd $ncollide_dir/$crate; cargo clean
done

echo "... documentation generated!"

./fix_rustdoc.sh
