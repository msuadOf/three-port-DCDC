# 2021年电赛C题-三端口DC-DC变换器

——记国赛训练题方案和调试过程

## 硬件方案

## 软件设计

整体上使用stm32 CubeMX+keil开发，main函数在`/Core/Src`目录下。

### 1.MPPT算法

#### （1）参考方案

[MPPT算法（恒定电压、扰动观察、电导增量）介绍与实现过程](https://blog.csdn.net/qq_27158179/article/details/82656494)

最终选用了比较基础的扰动观察法，有点震荡

#### （2）代码

- 先定义一堆结构体和函数

  ```c
  typedef struct
  {
    enum
    {
      MPPT_rising_step_len = 1,
      MPPT_falling_step_len = 1,
      MPPT_stable_step_len = 1,
    } step_len; // 步长
  
    // Power
    float Power_last;
    float Power_current;
    float delta_Power;
    float delta_Power_min_thereshold; // 配置参数
  
    // voltage
    float Voltage_last;
    float Voltage_current;
    float delta_Voltage;
    float delta_Voltage_min_thereshold; // 配置参数
  
    float out;
    float a; // 输出系数
  
    float step_size;
    float update_thereshold; // 因为adc有晃动，所以如果没有这个的话无法爬坡。
  
  } MPPT_t;
  
  MPPT_t mppt = {
      .a = -1000,
      .out = 65503,
      .delta_Voltage_min_thereshold = 0.01,
      .delta_Power_min_thereshold = 0.01,
      .step_size = -100,
      .update_thereshold = 0.02
  
  };
  
  float mppt_process(MPPT_t *mppt, float voltage, float power, float out_max, float out_min)
  {
    float step_size = 0;
    step_size = mppt->step_size;
    mppt->Power_current = power;
    mppt->Voltage_current = voltage;
  
    mppt->delta_Power = mppt->Power_current - mppt->Power_last;
    mppt->delta_Voltage = mppt->Voltage_current - mppt->Voltage_last;
  
    //  if (fabs(mppt->delta_Voltage) > mppt->delta_Voltage_min_thereshold)
    //  {
    //    mppt->out = (mppt->out) + (mppt->a) * (mppt->delta_Power / mppt->delta_Voltage);
    //  }
  
    if (fabs(mppt->delta_Power) > mppt->update_thereshold)
    {
      if (mppt->delta_Power > mppt->delta_Power_min_thereshold)
      {
        mppt->out = mppt->out + mppt->step_size;
      }
      else
      {
        if (mppt->delta_Power < -mppt->delta_Power_min_thereshold)
        {
          mppt->step_size = -mppt->step_size;
          mppt->out = mppt->out + mppt->step_size;
        }
      }
  
      // update paramenter
  
      mppt->Power_last = mppt->Power_current;
      mppt->Voltage_last = mppt->Voltage_current;
    }
  
    // 进行pwm输出限幅
    if (mppt->out > out_max)
    {
      mppt->out = out_max;
    }
    else if (mppt->out < out_min)
    {
      mppt->out = out_min;
    }
  
    return (mppt->out);
  }
  ```

- 然后在定时触发的ADC中调用，更新占空比

  ```c
  hhrtim1.Instance->sTimerxRegs[3].CMP1xR = 
      mppt_process(&mppt, V_battery, V_battery * I_battery,
                   0.98f * (hhrtim1.Instance->sTimerxRegs[0].PERxR),
                   0.3f * (hhrtim1.Instance->sTimerxRegs[0].PERxR));
  ```

  

#### (3) 代码解释

- 结构体定义

  屎山捏，里面有不少没用上的，比如enum变量

  ```c
  typedef struct
  {
    enum
    {
      MPPT_rising_step_len = 1,
      MPPT_falling_step_len = 1,
      MPPT_stable_step_len = 1,
    } step_len; // 步长
  
    // Power
    float Power_last;
    float Power_current;
    float delta_Power;
    float delta_Power_min_thereshold; // 配置参数
  
    // voltage
    float Voltage_last;
    float Voltage_current;
    float delta_Voltage;
    float delta_Voltage_min_thereshold; // 配置参数
  
    float out;
    float a; // 输出系数
  
    float step_size;
    float update_thereshold; // 因为adc有晃动，所以如果没有这个的话无法爬坡。
  
  } MPPT_t;
  
  ```

- 算法实现

  PWM正端接的上管，所以


$$
  Vo=Vi/Duty, 其中Duty简写为D
$$

  根据占空比和功率相等公式:

$$
  Pi=\frac{Vi^2}{Ri}=\frac{Vo^2}{Ro}
$$

  有boost输入电阻公式：

$$
Ri=D^2\times Ro
$$

  上式在调试的时候很有帮助。

  在占空比给满的情况下，输入阻抗是最大的，为输出电阻Ro。

  整个算法大致思路就是，你这个占空比从0.98慢慢开始往下降，然后你输入电阻也会减小，你的输入功率开始变大，然后当你发现接着减小占空比，功率却下降了，那就反向操作一手。

  下面这一部分代码就是每个mppt过程的初始化。

  输入了当前电压、功率、占空比上下限，然而目前算法中电压量并没有用到

  ```c
  float mppt_process(MPPT_t *mppt, float voltage, float power, float out_max, float out_min)
  {
    float step_size = 0;
    step_size = mppt->step_size;
    mppt->Power_current = power;
    mppt->Voltage_current = voltage;
  
    mppt->delta_Power = mppt->Power_current - mppt->Power_last;
    mppt->delta_Voltage = mppt->Voltage_current - mppt->Voltage_last;
  ```

  

  然后这是核心算法，如果大于Power变化的最小量的话（值比较大有助于最后状态稳定，但也会降低追踪点精度），就进行更改。

  变大了，说明之前调整方向正确，接着更改；变小了，说明过了，调个头回去，也就是`mppt->step_size = -mppt->step_size;`

  ```c
     if (mppt->delta_Power > mppt->delta_Power_min_thereshold)
      {
        mppt->out = mppt->out + mppt->step_size;
      }
      else
      {
        if (mppt->delta_Power < -mppt->delta_Power_min_thereshold)
        {
          mppt->step_size = -mppt->step_size;
          mppt->out = mppt->out + mppt->step_size;
        }
      }
  
  ```

  但是我们在调代码的时候发现，占空比卡在0.9以上下不来，好像有下来的意思，但是走走停停，速度很慢。

  经过分析发现，是adc在随机跳，导致Power值也在一定幅度跳，然后我处理速度有20kHz，还是比较快的，所以每次的 `delta_Power`值还是比较小的，甚至在爬坡阶段`delta_Power`会出现负值。所以最外层套了一个判断`if (fabs(mppt->delta_Power) > mppt->update_thereshold)`。这一段if的作用是这样的：如果我 `delta_Power`没在乱跳，我正常干活； `delta_Power`在乱跳，我这个循环的计算就省略了，而且我参数更新（把这一次的Power值保存）也不做，方便下一次 `delta_Power`的值能更大一些，防止间隔时间太短变化不大adc采不出来。

  这段代码两层if逻辑其实有重合，可以合并成一个，并把 `update_thereshold`参数给扔了。

  ```c
    if (fabs(mppt->delta_Power) > mppt->update_thereshold)
    {
      if (mppt->delta_Power > mppt->delta_Power_min_thereshold)
      {
        mppt->out = mppt->out + mppt->step_size;
      }
      else
      {
        if (mppt->delta_Power < -mppt->delta_Power_min_thereshold)
        {
          mppt->step_size = -mppt->step_size;
          mppt->out = mppt->out + mppt->step_size;
        }
      }
  
      // update paramenter
  
      mppt->Power_last = mppt->Power_current;
      mppt->Voltage_last = mppt->Voltage_current;
    }
  ```

  

- 参数配置

  首先设置一个初始输出out=65503，从高开始降嘛，所以step_size=-100得是负的，每次CCR降100
  
  `delta_Voltage_min_thereshold`这东西最好是要看下调试的时候，开环稳定情况下它会在多少范围跳
  
  ```c
  MPPT_t mppt = {
      .a = -1000,
      .out = 65503,
      .delta_Voltage_min_thereshold = 0.01,
      .delta_Power_min_thereshold = 0.01,
      .step_size = -100,
    .update_thereshold = 0.02
  
  };
  ```
  
  

### 2.pid算法

boost需要跑一个恒30V的pid

定时器中断进快点，拉到100kHz，然后对输入值做5个点的均值滤波，最终出来的采样率是20kHz。

不得不说STM32G474就是好用，速度拉高，pid都不用咋调的，符号选对，量级选对，adc校对，嘎嘎稳。

使用的是arm的dsp库, 效果比自己手搓的好，但没有积分抗饱和，我觉得有必要自己手搓。

```c
float pid_process(arm_pid_instance_f32 *pid, float aim, float current, float out_max, float out_min)
{
  float out = 0;

  arm_pid_init_f32(pid, 0);// 初始化为0

  error_inspector = (current - aim);

  out = arm_pid_f32(pid, (current - aim));

  // 进行pwm输出限幅

  if (out > out_max)
  {
    out = out_max;
  }
  else if (out < out_min)
  {
    out = out_min;
  }

  return out;
}
```

稍微解释下代码，`arm_pid_f32`arm的pid使用的pid计算参数不是Kp、Ki，而是奇奇怪怪的A1、A2，导致你在改动结构体中Kp、Ki后无法实时更新。在查阅了官方api（有讲解使用了什么模型，参数啥意思），并且扒了函数实现，如果想要实现调试时实时更新参数（也许这也是调试时屎山的体现），需要每个周期都重新算一遍A参数，为了简洁调用init函数`arm_pid_init_f32`，并且第二个参数重置flag设为0（也许arm写的时候也是这么想的），不让他重置state值（可以理解为积分项）。

```c
arm_pid_instance_f32 PID_Voltage = {
    .Kp = -100,
    .Ki = 100,
    .Kd = 0
};
```

```c
int main(){
    ...
    arm_pid_init_f32(&PID_Voltage, 1);
    ...
}
```

