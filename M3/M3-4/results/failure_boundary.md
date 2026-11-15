# 失效边界：实测触发实验

有限重复只能支持该场景中的观测，不证明任意噪声下必然失败。导航终止状态 6=ABORTED；超时单独记录。

|场景|次数|成功|障碍世界坐标|尺寸 m|实际注入 sim s|终止耗时 sim s|终止原因|
|---|---:|---:|---|---:|---|---|---|
|blocked_1p6|3|0|(2.5,-5.05)|1.6|3.10–3.20|10.90–11.00|action_status_6|
|large_0p8|3|0|(2.5,-5.05)|0.8|3.00–3.20|14.60–15.70|action_status_6|
|near_1p4|3|0|(1.4,-5.05)|0.6|3.10–3.20|8.90–9.00|action_status_6|

与标准条件0.60m、x=2.5m的A方案9/10相比，上述记录给出了可脚本重现的失效触发条件。当前尺寸只采样0.60/0.80/1.60m、距离只采样1.4/2.5m，不构成连续精确阈值搜索；不能声称更小障碍必成功或任意更近位置必失败。

以下为各条件的逐次结果；三组均使用A方案、同一目标和3秒请求延迟。

南侧通道几何宽度 2.10m。中心障碍留下的单侧几何宽度=(2.10-size)/2；物理车宽0.22m，导航 footprint 宽0.30m。膨胀半径0.40m是代价衰减带，不能直接当成不可通行硬边界。前进转弯半径≥0.60m，纵向准备距离也影响可行性。

1.60m障碍留下单侧0.25m，小于导航footprint宽0.30m，因此南侧就近绕行不满足配置中的车体净宽；这不证明整个环形地图无解。可能存在更长的全局绕行，超时只表示本配置在90仿真秒预算内未到达。近距离场景检验的是前进转弯准备距离，不能简单归因为障碍面积。

## 逐次结果

|场景|试次|结果|导航耗时 sim s|路径长度 m|实际注入 sim s|
|---|---:|---|---:|---:|---:|
|blocked_1p6|1|ABORTED|10.900|1.074983|3.200|
|blocked_1p6|2|ABORTED|10.900|1.069289|3.100|
|blocked_1p6|3|ABORTED|11.000|1.082435|3.200|
|large_0p8|1|ABORTED|14.700|2.065346|3.000|
|large_0p8|2|ABORTED|14.600|1.987753|3.100|
|large_0p8|3|ABORTED|15.700|2.169147|3.200|
|near_1p4|1|ABORTED|9.000|0.710884|3.100|
|near_1p4|2|ABORTED|8.900|0.712163|3.100|
|near_1p4|3|ABORTED|8.900|0.709491|3.200|

## 复现命令

```bash
python3 scripts/run_benchmark.py --planners A --runs 3 --obstacle-size 0.8 --out results/boundary-large
python3 scripts/run_benchmark.py --planners A --runs 3 --obstacle-size 1.6 --out results/boundary-blocked
python3 scripts/run_benchmark.py --planners A --runs 3 --obstacle-at 1.4,-5.05 --out results/boundary-near
```
