A板遥控器DBUS是UART1,C板是UART3波特率为100000,数据长8位,带校验位位9位,偶校验(EVEN)
用户LED为PE11（LED_R),PF14（LED_G);PG1~PG8(都是绿色）
A板和C板外部晶振都是是12MHZ;
配置外设是要先开启外设才能开启中断，如：
HAL_CAN_Start(&hcan1);
__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
A板上的拨动开关是改变两个八路PWM输出的电压;
A板左边PWM输出A~H为TIM4和TIM5的CH1~4;右边Z~S，从上到下为TIM2和TIM8的CH1~4;
底盘电机通信通常为CAN1;云台为CAN2
当板子是A板时主频最高180MHZ，CAN的配置为：
	   		prescale  						    5 ;
			time quanta in bit segment 1    	2 Times;
			time quanta in bit segment 1     	6 Times;
			resychronization jump width     	1 Time;
			Time triggered Communication Mode   Disable;
			Automatic Bus_Off Management  		Enable;
			Atuomatic Wake_Up Mode              Enable;
			Automatic Retransmission  			Enable;
			Receive Fifo Locked Mode         	Disable;
			Transmit Fifo Priority				Enable;
若是C板则主频最高168MHZ，CAN的配置为：
	   		prescale  							3 ;
			time quanta in bit segment 1    	10 Times;
			time quanta in bit segment 1     	3 Times;
			resychronization jump width     	1 Time;
			Time triggered Communication Mode   Disable;
			Automatic Bus_Off Management  		Enable;
			Atuomatic Wake_Up Mode              Enable;
			Automatic Retransmission  			Enable;
			Receive Fifo Locked Mode         	Disable;
			Transmit Fifo Priority				Enable;
CAN2是依托于CAN1的时钟的，如果单独使用CAN2要开启CAN1时钟；使用CUBEMX或CUBEIDE时，单独使用系统会自动生成开启时钟的代码
开启两个CAN时要设置从机的过滤器编号filter_structure.SlaveStartFilterBank=14;作用是将前14个(0-13)过滤器分配给CAN1
在初始化CAN2过滤器时过滤器编号也要改filter_structure.FilterBank = 14;(14-27都可以)

PID控制：角度控制用双环，外环是角度环，内环是速度环
PID优化：1，积分限幅 2，积分分离 3，微分先行
PID双环：副回路需要积分作用， 主回路只需比例作用
/************************************************************************/
个人配置
/**************************************************************************/
用TIM2作为执行任务的定时器；中断为1ms;
头文件之间不要互相引用(如果需要在不同的文件中使用同样的宏、结构体、枚举等，可以将其放在无需引用其它头文件的文件中)