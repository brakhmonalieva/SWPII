int count = 0;
int toggle = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(LED_BUILTIN, toggle);

}
void loop() {
  Serial.println(++count);
  toggle = !toggle;
  digitalWrite(LED_BUILTIN, toggle);
  toggle = !toggle;
  delay(1000);

}
int toggle_state(int toggle) {
  return toggle;

}
