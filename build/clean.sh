GLOBIGNORE=*.sh:bin
rm -rf *
unset GLOBIGNORE

cd ./bin
./clean.sh
cd ../
