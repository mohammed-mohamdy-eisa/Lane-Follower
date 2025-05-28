
# الشرح باللهجة المصرية للكود بتاع الروبوت اللي بيفollow الخطوط  

## 1. التعليقات الأولية والإعدادات (الأسطر 1–42)  
```cpp
// روبوت بيفollow خطوط بترتيب حرف V للسينسورز ... (الأسطر 1–8)
int IN1 = 2; // دبابيس الموتور (الأسطر 9–17)
int thresholdValue = 200; // قيمة تمييز اللون الأسود عن الأبيض (الأسطر 18–20)
int baseSpeed = 70; // سرعة الموتورز (الأسطر 22–25)
float leftMotorFactor = 0.85; // ضبط توازن الموتور (الأسطر 27–28)
float Kp = 1.0, Ki = 0.003, Kd = 2.0; // إعدادات PID (الأسطر 31–34)
bool is90DegreeTurn = false; // علامات للدوران (الأسطر 36–37)
#define STATE_IDLE 0 // تعريف حالات الروبوت (الأسطر 39–51)
```

- **الغرض**: تحدد مواصفات الهاردوير (مسافات السنسورز، عرض المسار) والمتغيرات الأولية.  
- **ملاحظات مهمة**:  
  - السنسورز مرتبة على شكل **حرف V** (السنسورز الداخلية متباعدة 6 سم، الخارجية 16 سم).  
  - `thresholdValue = 200` بيفرق بين اللون الأسود (≥200) والأبيض (<200).  
  - `Kp`, `Ki`, `Kd` بيضبطوا شدة استجابة الروبوت للانحراف.  

---

## 2. دوال تحكم الموتور (الأسطر 44–78)  
```cpp
void rotateLeft(int speed) { 
  analogWrite(ENA, speed * leftMotorFactor);
  analogWrite(ENB, speed * rightMotorFactor);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void rotateRight(int speed) {
  analogWrite(ENA, speed * leftMotorFactor);
  analogWrite(ENB, speed * rightMotorFactor);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
```

- **بتعمل إيه؟**:  
  - `rotateLeft/Right()`: بتخلي الموتورز يدوروا في اتجاهين متعاكسين للدوران الحاد.  
  - `stopMotors()`: بتوقف الموتورز كلها.  
- **المنطق الأساسي**:  
  - سرعة الموتورز مضروبة في `leftMotorFactor`/`rightMotorFactor` عشان تصحح أي خلل في الموتورز.  
  - `constrain()` بتأكد أن السرعة متعديش 255 (أعلى قيمة لبWM).  

---

## 3. PID Controller (الأسطر 80–161)  
```cpp
void calculatePID() {  
  error = 0;  
  if (LEFT1 && LEFT2 && !RIGHT1 && !RIGHT2) {
    // كشف دوران 90 درجة
    is90DegreeTurn = true;
    turnStartTime = millis();
  }
  else { // حالة المتابعة العادية
    if (LEFT1) error -= 4; // السنسور الخارجي ليه تأثير أقوى  
    if (RIGHT1) error += 4;  
  }  
  derivative = error - lastError;
  pidOutput = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  integral += error;
}
```

- **بتشتغل إزاي؟**:  
  - **حساب الخطأ**: لو الخطأ موجب يبقى الروبوت ناحية اليمين (يخش لليسار)، لو سالب يبقى ناحية اليسار (يخش لليمين).  
  - **حالات خاصة**: لو الاتنين سنسور داخلية شايفين خط أسود → يبقى في دوران 90 درجة.  
  - **خرج PID**: بيجمع بين التعديل السريع (P)، التعديل التدريجي (I)، والتخفيف من التمايل (D).  

---

## 4. دالة الإعداد setup (الأسطر 163–239)  
```cpp
void setup() {  
  pinMode(IN1, OUTPUT); // دبابيس الموتور
  pinMode(Lsensor1, INPUT); // دبابيس السنسور
  Serial.begin(115200); // التصحيح
  
  // ومضات الليد
  for(int i=0; i<3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  delay(5000); // انتظار 5 ثواني قبل البدء
}
```

- **الغرض**: بتهيئ الهاردوير وتبدأ السيريال مونيتور.  
- **مميزات مهمة**:  
  - الليد بيومض أثناء التشغيل عشان تتأكد أن كل حاجة شغالة.  
  - فيه 5 ثواني تأخير عشان تقدر تحط الروبوت على المسار.  

---

## 5. اللوب الرئيسي loop (الأسطر 241–591)  
### **الجزء 5.1: قراءة السنسورز (الأسطر 243–250)**  
```cpp
RIGHT1 = (analogRead(Rsensor1) > thresholdValue) ? 1 : 0;
LEFT1 = (analogRead(Lsensor1) > thresholdValue) ? 1 : 0;
```

- **ببساطة**: بتحول قراءة السنسور من إشارة تماثلية (0–1023) لرقم ثنائي (1=أسود، 0=أبيض).  

### **الجزء 5.2: حالات خاصة (الأسطر 252–215)**  
```cpp
if (RIGHT1 && RIGHT2 && LEFT1 && LEFT2) { 
  stopMotors(); // لو كل السنسورز شايفة أسود -> قف
}
if (!RIGHT1 && !RIGHT2 && !LEFT1 && !LEFT2) { 
  moveForward(); // لو كلها أبيض -> امشي
}
```

- **المنطق**:  
  - **كل السنسورز سودا**: يبقى في تقاطع -> وقف كامل.  
  - **كل السنسورز بيضا**: يبقى الروبوت في المنتصف -> امشي قدام.  

### **الجزء 5.3: التحكم PID (الأسطر 317–366)**  
```cpp
calculatePID();  
int leftSpeed = baseSpeed - pidOutput;
int rightSpeed = baseSpeed + pidOutput;
moveForward(leftSpeed, rightSpeed);
```

- **إيه اللي بيحصل؟**: بيطبق خرج PID على الموتورز (كل ما زاد `pidOutput` كل ما الدوران يكون أشد).  

### **الجزء 5.4: التعامل مع الدوران (الأسطر 284–316)**  
```cpp
if (isTurning && millis() - turnStartTime < turn90Duration) {  
  rotateLeft(turnSpeed); // دوران بقياس الوقت
}
```

- **ليه الدوران بالتايمر؟**: عشان يضمن أن الدوران يكمل 90 درجة حتى لو السنسورز مشفتش الخط في المنتصف.  

### **الجزء 5.5: التصحيح (الأسطر 368–425)**  
```cpp
void printRobotStatus() {  
  Serial.print("اليسار: "); Serial.print(leftVal1);
  Serial.print(" اليمين: "); Serial.print(rightVal1);
  Serial.print(" الحالة: ");
  if(isTurning) Serial.println("دوران");
  else Serial.println("متابعة");
}
```

- **استخدامها**: بتبعث بيانات السنسورز والحالة على السيريال مونيتور عشان لو في مشكلة تعرف سببها.  

### **الجزء 5.6: دوال مساعدة (الأسطر 427–591)**  
```cpp
void moveForward(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  analogWrite(ENA, leftSpeed * leftMotorFactor);
  analogWrite(ENB, rightSpeed * rightMotorFactor);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
```

- **تفصيل صغير**: بتضبط سرعة الموتورز حسب عوامل التصحيح قبل ما تحركهم.  

---

### **نصائح للمبتدئين**  
1. **ترتيب السنسورز**: شكل الـ V بيخلي الروبوت يحس بالمنعطفات بدري.  
2. **ضبط PID**: غير في `Kp`, `Ki`, `Kd` عشان توازن بين السرعة والثبات.  
3. **حالات الروبوت**: بيتابع إذا كان بيدور، واقف، إلخ. عشان الحركة تكون سلسة.  
4. **التصحيح**: استخدم `printRobotStatus()` عشان تفهم ليه الروبوت بيعمل حركة غريبة.  

**جرب دلوقتي**: ابدأ بتغيير `baseSpeed` (السطر 22) و `Kp` (السطر 31) عشان الدوران يكون أهدأ.
```
