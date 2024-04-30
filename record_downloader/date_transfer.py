# 打开文件
file_path = "date_data.txt"
date = ""
# "2024-04-11 15:44:30", "2024-04-11 15:46:00"
# 9:53-9:55
with open(file_path, "r") as data_source:
    with open('output.txt', "w") as out_file:
        for line in data_source:
            if("date" in line):
                line = line.strip()
                date = line.split(":")[1]
            else:
                line = line.strip()
                start_time,end_time = line.split("-")
                print(f"['{date} {start_time}:00',{date} {end_time}:00']")
                out_file.write(f"['{date} {start_time}:00','{date} {end_time}:00'],\n")