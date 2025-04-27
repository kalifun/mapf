`CBS(Conflict-Based Search) `是一种基于冲突的`MAPF(Multi-Agent Path Finding)`算法, `CBS` 算法给出 `MAPF` 问题的全局最优结果。

## 概述

`CBS` 是一种中央规划算法`(Centralized Planning for Distributed Plans)`。由一台主机作为中央控制器，在全局视角生成每一台机器人的路径并统筹解决冲突。

## 思想

将多机规划分为`两层`，`底层`执行带有约束的单机规划(使用传统的A\*)，`顶层`遍历底层的规划路径，检查路径之间是否有冲突，如果有冲突则施加约束重新进行单机规划，直到无冲突为止。

## 术语

- $path$ : 一条机器人的路径，也就是我们平时用得最多的单机规划结果
- $solution$ : 多机系统中所有机器人的 $path$ 的集合( $n$ 条 $path$ )，也就是 mapf 算法的全局规划结果
- $conflict$ : 冲突．上述的 $solution$ 中， $n$ 条 $path$ 之间可能会有冲突(没冲突当然皆大欢喜了)．具体的描述形式为 $(a_i, a_j, v, t)$，表示在时刻 $t$ , $a_i$ 和 $a_j$ 同时占据了顶点 $v$ . 拿栅格地图来说，就是在时刻 $t$ , $a_i$ 和 $a_j$ 同时占据了矩阵的一个格子 $matrix(i, j)$ .
- $constraints$ : 约束．一个约束 $(a_i, v, t)$，表示在时刻 $t$ , $a_i$ 不能占据顶点 $v$ .

## 参考

参考文献:
[Conflict-based search for optimal multi-agent path finding](https://link.zhihu.com/?target=https%3A//doi.org/10.1016/j.artint.2014.11.006)
