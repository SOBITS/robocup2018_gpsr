__init__.py の中にimportしたいファイルを適宜記述すること
「from . import *」だとうまくimportされないので注意。

フォルダ全体をimportする場合は、
from フォルダ名 import *
とすれば良い。


