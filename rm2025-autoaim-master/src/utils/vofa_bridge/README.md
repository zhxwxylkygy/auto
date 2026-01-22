# Vofa Bridge

## 简介

Vofa+ 上位机协议调试用封装库。

## 用途

实现了基于UDP协议的JustFloat数据引擎格式，用于使用Vofa+上位机进行调试数据展示与绘图。

## 用法

```cpp
VofaBridge vb = VofaBridge::Get();
for (auto&& i = 1; i < 500; ++i) {
    auto num = static_cast<float>(i) / 10.f;
    vb.PushData(std::sin(num));
    vb.PushData(std::cos(num));
    vb.PushData(std::tan(num));
    vb.SendData();
    std::this_thread::sleep_for(100ms);
}
std::this_thread::sleep_for(1000ms);
vb.ClearData();
for (auto&& i = 1; i < 500; ++i) {
    auto num = static_cast<float>(i) / 10.f;
    vb.AssignData(std::sin(num), std::cos(num), std::tan(num));
    vb.SetData(3, i);
    vb.SendData();
    std::this_thread::sleep_for(100ms);
}
std::this_thread::sleep_for(1000ms);
vb.ClearData();
vb.SendOnce(3, 1, 4);
std::this_thread::sleep_for(1000ms);
vb.SendOnce(6, 2, 8);
std::this_thread::sleep_for(1000ms);
vb.SendOnce(9, 4, 2);
```

## API

```cpp
static VofaBridge& Get();
```

获取一个VofaBridge实例引用。

```cpp
void Reset(std::string_view ip, unsigned short port);
```

重置与vofa+的连接。

```cpp
void SendOnce(auto&&... data);
```

直接发送一次数据。

```cpp
void PushData(auto&&... data);
```

向缓冲区尾部插入指定数据。

```cpp
void AssignData(auto&&... data);
```

重新赋值缓冲区所有数据。

```cpp
void SetData(std::size_t index, auto&& data);
```

将缓冲区指定下标位置设置为给定数据。

```cpp
void SendData();
```

发送缓冲区内的所有数据。

```cpp
void ClearData();
```

清空缓冲区内的所有数据。

## 注意事项

Vofa+请使用JustFloat数据引擎与UDP数据接口。
