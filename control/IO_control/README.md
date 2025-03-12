Installing
----------

```
sudo python -m pip install --upgrade pip setuptools wheel
pip install image
```
This code uses this lib
```
sudo pip install Adafruit-SSD1306
```

or
```
git clone https://github.com/adafruit/Adafruit_Python_SSD1306.git
cd Adafruit_Python_SSD1306
sudo python setup.py install
```
Checking connection
----------
```
sudo usermod -aG i2c <username>
sudo i2cdetect -y -r 1
```
After installing, copy `oled_msg.py` to `Adafruit_Python_SSD1306` folder and run there