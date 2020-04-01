***模型说明文档  ——张仕祥 2020/03/12 ***
模型在项目名 TJ_Project 下 Invener_Model 文件夹中，其中 KUKA_BP为库卡机器人模型（蓝图类Actor型），FrameStruct_BP为整体模型（蓝图类Actor型），其中库卡机器人调用自KUKA_BP。KukaInterface为预留的蓝图接口，当前版本还未添加联系与功能。

欲调整KUKA机器人机械臂位置角，则调整 KUKA_BP 下对应零件的旋转角度，其层级关系已经建立完毕，次级会跟随移动。
欲调整KUKA机器人水平移动位置，则调整 FrameStruct_BP 下组件最后 一项 KUKA_BP 的 Y值即可。原值不为零，默认值见注2_0编号值。

零件在 Invener_Model 下两个文件夹中。
演示关卡在Content 下 TestMap 的 ZSX_testMap，测试功能未添加。

注：
1.obj文件在 zsx_e20200310 下 inventer_model。
2.初始偏移值在 zsx_e20200310 下 kuka_初始位置偏差表.txt ，其中0编号值对应水平方向的位置偏移初始值。
3.盒子在 FrameStruct_BP 下 FrameStruct_2_BOX，可移动。
4.碰撞模型当前为导入时自动生成的，包括简单碰撞与复杂碰撞模型，复杂碰撞模型因零件数量较多，开销较大，建议区别使用。
5.inventer模型默认单位为mm，ue4默认cm，ZSX_testMap 中使用时缩小了10倍。喜欢调整设置1U=1mm，也是可以的。与其他项目有关联的话，建议统一。
6.KUKA机器人限位角未添加，不拥有对应技术文档，注意KUKA_BP 下 kuka_5 与 kuka_6 易碰撞。
7。模型版本同20200113版，至当前未有添加及扩展。kuka 模型经沟通做出了相应的删减。（未核实）


附：
1.建议不要轻易更改默认值，使用接口读取备份后在其上附加偏移量进行更改。
2.出于系统/硬件问题，ZSX_testMap打开困难或报错，可删除 Content 下 ThirdPersonBP 与 TestMap文件夹，以及 StarterContent 文件夹，但要注意保留 StarterContent 下 Materials 下 M_Glass 材质。并自建关卡。
3.4.24版本更改编译光照的默认位置未能找到，文件夹中不包含构建光的缓存文件，初次打开可能要花费一定量时间（几分钟视性能）进行编译。不看光的话不想编译也行。
4.不建议渲染质量开得过高。
5.实际运行时静态的模型以设为静态。
