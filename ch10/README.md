## 编译代码中的一些问题
1. 老问题了，需要在CMakeList.txt里面加上
```
set(CMAKE_CXX_STANDARD 14)
```

2. 在终端直接执行g2o_viewer即可打开此软件，在后面加.g2o文件名即可打开此文件，例如：
```
g2o_viewer result.g2o
```
