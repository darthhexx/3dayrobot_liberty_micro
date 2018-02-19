#include "JetsonCommand.h"

class JetsonCom {
public:
	bool update();
	char buffer[50];
	int buffer_count = 0;
  JetsonCommand command;
};

/* Yes this is a hack of a CSV parser but shush */
bool JetsonCom::update() {
	if (Serial2.available()) {
		buffer[buffer_count++] = Serial2.read();

    Serial.print(buffer[buffer_count]);

		if (buffer[buffer_count - 1] == '\n') {
			// End of line parse please
			char buf[10];
			int buf_cnt;
      int howmanycsv = 0;
			for (int i = 0; i < buffer_count; i++) {
				if (buffer[i] != ',' && i != buffer_count - 1) {
					buf[buf_cnt++] = buffer[i];
				} else {
					buf_cnt = 0;
					double val = atof(buf);
          if (howmanycsv == 0) {
            command.dead = val;
            Serial2.print("Dead: ");
            Serial2.println(val);
          } else if (howmanycsv == 1) {
            command.speed = val;
            Serial2.print("Speed: ");
            Serial2.println(val);
          } else if (howmanycsv == 2) {
            command.angle = val;
            Serial2.print("Angle: ");
            Serial2.println(val);
          } else if (howmanycsv == 3) {
            command.sensor_angle = val;
          }
          howmanycsv++;
				}
			}
      buffer_count = 0;
      return true;
		} else {
			return false;
		}
	}
}
