class MyClass:
    def __init__(self, raw_imu):
        self._raw_imu = raw_imu  # Private attribute

    @property
    def raw_imu(self):
        return self._raw_imu  # Getter for the private attribute
    @raw_imu.setter
    def raw_imu(self, value):
        if value <0 :
            raise ValueError("raw_imu must be non-negative")
        self._raw_imu=value

# Create an instance of MyClass
my_instance = MyClass(10)

# Access the raw_imu property
print(my_instance.raw_imu)  # Output: 10
my_instance.raw_imu=20
print(my_instance.raw_imu)
my_instance.raw_imu = -5