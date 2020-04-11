![build](https://github.com/sssssssuzuki/autd3-for-practice/workflows/build/badge.svg)
![lint](https://github.com/sssssssuzuki/autd3-for-practice/workflows/lint/badge.svg)

# autd3 練習用リポジトリ

井上さんのオリジナルの実装 (https://github.com/shinolab/autd) をもとに, 必要最低限の機能のみに限定したもの.

Hardwareのversionはv0.4に対応.

オリジナルからの変更点は以下の通り

* Eigen3, boostの別途インストールを不要に
* SOEMのサポートを追加
* GainはFocalPointGainのみに
* ModulationはSineModulationのみに
* v0.4に対応
* cpplintに従って一部コードを修正

## Build

```
git clone https://github.com/sssssssuzuki/autd3-for-practice.git --recursive
cd client
mkdir build
cd build
cmake ..
make
```
