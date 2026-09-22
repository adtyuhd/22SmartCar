# 1.运行
	cd ~/22SmartCar/M0/M0-3
	python3 sensor_analyzer.py
# 2.error 1
 发现报错：File "/home/adtyuhd/22SmartCar/M0/M0-3/sensor_analyzer.py", line 37,  in <module>
    v = float(row["Value"])
    KeyError: 'Value'
       问题是‘V’大小写错误，应为'value'
# 3.error 2
可以读取数据，但是无法输出
发现报错： File "/home/adtyuhd/22SmartCar/M0/M0-3/sensor_analyzer.py", line 62, in <module>
    f = open(output_path, "w")
FileNotFoundError: [Errno 2] No such file or directory: '/out/cleaned_data.csv'
问题是path错误和out目录可能不存在
代码修正为：
		os.makedirs(OUTPUT_DIR, exist_ok=True)
		output_path = os.path.join(OUTPUT_DIR, OUTPUT_FILE)
# 4.error 3
输出：
		=== 传感器数据分析 ===
		共读取 60 条数据
		均值 mean = 50.4315
		标准差 std = -0.0000
		清洗后剩余 0 条
		已保存到 out/cleaned_data.csv
std的结果存在明显错误，推测代码逻辑存在错误
代码修正为：
		for v in data:
		    acc += (v - mean)*(v - mean)
		std = math.sqrt(acc / len(data))
# 5.error 4
输出：
		=== 传感器数据分析 ===
		共读取 60 条数据
		均值 mean = 50.4315
		标准差 std = 18.5033
		清洗后剩余 0 条
		已保存到 out/cleaned_data.csv
代码逻辑和“是"超过 2 倍标准差"的绝对值判定”不符
代码修正为：
		for v in data:
		    acc += (v - mean)*(v - mean)
		std = math.sqrt(acc / len(data))
# 6.error 5
输出：
		=== 传感器数据分析 ===
		共读取 60 条数据
		均值 mean = 50.4315
		标准差 std = 18.5033
		清洗后剩余 57 条
		已保存到 out/cleaned_data.csv
但是查看cleaned_data.csv发现只有一列，数据输出格式错误
同时remove的用法存在问题
代码修正为：
		for t, v in zip(times, data):
		    if abs(v - mean) <= 2 * std:
			cleaned.append((t, v))
		------------------------------
		for t,v in cleaned:
    			writer.writerow([t,v])
# 7. error 6
发现代码写死绝对路径
代码修正：cleaned_data.csv 默认写到当前工作目录，同时处理input和output
# 8. error 7
数据读取报错处理，不出现Traceback
代码修正处理：
		try:
		    f = open(input_path, "r")
		except FileNotFoundError:
		    print("Error: 输入文件不存在:", input_path)
		    sys.exit(1)

		reader = csv.DictReader(f)

		# 空文件：连表头都没有
		if reader.fieldnames is None:
		    print("Error: 输入文件为空")
		    f.close()
		    sys.exit(1)

		# 检查列名
		if "time" not in reader.fieldnames or "value" not in reader.fieldnames:
		    print("Error: CSV 必须包含 time 和 value 两列")
		    f.close()
		    sys.exit(1)

		for line_number, row in enumerate(reader, start=2):
		    try:
			t = float(row["time"])
			v = float(row["value"])
		    except (ValueError, TypeError):
			print("Error: 第 %d 行包含非数值内容" % line_number)
			f.close()
			sys.exit(1)

		    times.append(t)
		    data.append(v)

		f.close()

		# 有表头，但没有任何数据行
		if len(data) == 0:
		    print("Error: CSV 没有数据")
		    sys.exit(1)
# 9. error 8
代码缺陷：缺失f.close()和多了一行f = open(output_path, "w")，修正

