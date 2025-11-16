你可以用 **pip** 直接安装 `.whl` 包，只需要保证文件路径正确即可。

---

# ✅ **安装 .whl 的标准方法**

假设你的 wheel 文件在当前目录：

```bash
pip install /deps/msgpack_rpc_python-0.4-py3-none-any.whl
```

如果你使用 Python3：

```bash
pip3 install /deps/msgpack_rpc_python-0.4-py3-none-any.whl
```


同理：

pip3 install airsim-1.8.1-py3-none-any.whl



注意，一定先安装 msgpack_rpc_python-0.4-py3-none-any.whl 再安装 airsim-1.8.1-py3-none-any.whl

如果你希望使用venv




# ✅ **1. 创建并启用 venv（推荐用 Python3 自带的 venv）**

进入你想放虚拟环境的目录，例如：

```bash
cd ~/flyer_proj
```

创建虚拟环境：

```bash
python3 -m venv airsim_env
```

激活环境：

```bash
source airsim_env/bin/activate
```

激活后你会看到终端前缀变成：

```
(airsim_env) user@pc:~$
```

---

# ✅ **2. 在 venv 中升级 pip（重要）**

```bash
pip install --upgrade pip
```

---

# ✅ **3. 安装 wheel 文件**

假设你把 `.whl` 文件放在当前目录（如果路径不一样，改成你的路径）：

---

### **① 安装 msgpack_rpc_python（必须先安装）**

```bash
pip install /deps/msgpack_rpc_python-0.4-py3-none-any.whl
```

---

### **② 再安装 airsim**

```bash
pip install /deps/airsim-1.8.1-py3-none-any.whl
```

---

# ✅ **4. 验证是否安装成功**

进入 Python：

```bash
python
```

然后：

```python
import airsim
import msgpackrpc
```

没有报错就成功了。

退出：

```python
exit()
```

---

# 📌 **5. 停用虚拟环境（不需要时）**

```bash
deactivate
```

---

