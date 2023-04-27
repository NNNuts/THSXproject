MoveJ :
先使用Tar.PathInit()初始化路径
再使用Tar.SetPathPoint()设置路径点，最多不超过10个
然后使用Tar.Move_MoveJ_CubicPolynomial()开始MoveJ运动

MoveL：
先使用Tar.PathInit()初始化路径
再使用Tar.SetPathPoint()设置路径点，最多不超过10个
然后使用Tar.Move_End_LadderShaped()开始MoveJ运动

目标再引导：
先使用Tar.TrackInit()初始化当前位姿
再用Tar.trackMod = true启动路径计算
然后使用Tar.setTarget()设置目标点，
运行中途可以重新设置目标点