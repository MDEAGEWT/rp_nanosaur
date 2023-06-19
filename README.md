## 알고리즘 
1. 선 중앙 탐색
```
    for i in range(0,len(end_line)):
        if end_line[i] == 0 and start == -1:
            start = i
        if end_line[i] == 255 and start != -1 and end == -1:
            end = i

    if end == -1:
        end = len(end_line) - 1

    pos = start + ((end - start ) // 2)

    center = len(end_line) // 2

    print(start, end, pos, center)
    global stop
    global last_z
```
2. 선의 위치에 따라 방향 결정

```
    # 선을 못 찾으면 후진
    if start == -1:
        print("후진")
        message.linear.x = -0.03
        message.linear.z = last_z * -1.0
    elif pos >= center - 30 and pos <= center + 30:
        print("직진")
        message.linear.x = 0.03
    elif pos <= center - 15:
        stop = stop + 1
        print("턴1")
        message.linear.x = 0.02
        message.angular.z = 0.2
        last_z = 0.2
    else:
        stop = stop + 1
        print("턴2")
        message.linear.x = 0.02
        message.angular.z = -0.2
        last_z = -0.2
```
