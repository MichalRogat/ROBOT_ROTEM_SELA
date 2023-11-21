def TelemetricInfoSend(self):
        info = {
            "opcode": CommandOpcode.telemetric.name,
            "imu-1": np.subtract(np.array(self.angle1), np.array(self.offset1)).tolist(),
            "imu-2": np.subtract(np.array(self.angle2), np.array(self.offset2)).tolist(),
            "imu-3": np.subtract(np.array(self.angle3), np.array(self.offset3)).tolist(),
            "imu-4": np.subtract(np.array(self.angle4), np.array(self.offset4)).tolist(),
            "imu-5": np.subtract(np.array(self.angle5), np.array(self.offset5)).tolist(),
            "drive1": self.a2d.values[1],
            "drive2": self.a2d.values[2],
            "elev": self.a2d.values[4],
            "turn1": self.a2d.values[3],
            "turn2": self.a2d.values[5],
            "joint1": self.a2d.values[6],
            "FullTank1": self.a2d.values[7],
            "FullTank2": self.a2d.values[8],
            "FullTank3": self.a2d.values[9],
            "activePump": self.activePump,
            "pumpingNow": self.isPumpingNow,
            "Spare2": 4096,
            "Spare3": 4096,
            "Spare4": 4096,
            "Spare5": 4096,
            "Spare6": 4096,
            "Spare7": 4096,
            "Camera-F1": True,
            "Camera-S1": True,
            "Camera-F2": True,
            "Camera-S2": True,
            "Camera-F3": True,
            "Camera-S3": True,
            "Camera-F4": True,
            "Camera-S4": True,
            "isFlip": self.isFlip,
            "isToggle": self.isToggle
        }

        while True:
            try:
                t = threading.Thread(target=GenericFunctions.callReadNano, args=(Entity.ITrailer.trailer_instances, info))
                t.start()
            except Exception as e:
                print(e)
            finally:
                time.sleep(0.5)

                