int IN_1D = 2; //depan kiri
int IN_2D = 3;
int IN_3D = 4; //depan kanan
int IN_4D = 5;
int IN_1B = 6; //belakang kanan
int IN_2B = 7;
int IN_3B = 8; //belakang kiri
int IN_4B = 9;

int fast = 120; //kecepatan default (max kecepatan) 255

void setup_motor() {
  pinMode(IN_1D, OUTPUT);
  pinMode(IN_2D, OUTPUT);
  pinMode(IN_3D, OUTPUT);
  pinMode(IN_4D, OUTPUT);
  pinMode(IN_1B, OUTPUT);
  pinMode(IN_2B, OUTPUT);
  pinMode(IN_3B, OUTPUT);
  pinMode(IN_4B, OUTPUT);

  digitalWrite(IN_1D, LOW);
  digitalWrite(IN_2D, LOW);
  digitalWrite(IN_3D, LOW);
  digitalWrite(IN_4D, LOW);
  digitalWrite(IN_1B, LOW);
  digitalWrite(IN_2B, LOW);
  digitalWrite(IN_3B, LOW);
  digitalWrite(IN_4B, LOW);
}

void maju() {
  analogWrite(IN_1D, 0);
  analogWrite(IN_2D, fast);
  analogWrite(IN_3D, 0);
  analogWrite(IN_4D, fast);
  analogWrite(IN_1B, 0);
  analogWrite(IN_2B, fast);
  analogWrite(IN_3B, fast);
  analogWrite(IN_4B, 0);
  isMovingForward = true;
  isTurningLeft = false;
  isTurningRight = false;
  isMovingBackward = false;
  isRotatingLeft = false;
  isRotatingRight = false;
  isSwipeLeft = false;
  isSwipeRight = false;
}

void mundur() {
  analogWrite(IN_1D, fast);
  analogWrite(IN_2D, 0);
  analogWrite(IN_3D, fast);
  analogWrite(IN_4D, 0);
  analogWrite(IN_1B, fast);
  analogWrite(IN_2B, 0);
  analogWrite(IN_3B, 0);
  analogWrite(IN_4B, fast);
  isMovingForward = false;
  isTurningLeft = false;
  isTurningRight = false;
  isMovingBackward = true;
  isRotatingLeft = false;
  isRotatingRight = false;
  isSwipeLeft = false;
  isSwipeRight = false;
}

void stop(){
  digitalWrite(IN_1D, LOW);
  digitalWrite(IN_2D, LOW);
  digitalWrite(IN_3D, LOW);
  digitalWrite(IN_4D, LOW);
  digitalWrite(IN_1B, LOW);
  digitalWrite(IN_2B, LOW);
  digitalWrite(IN_3B, LOW);
  digitalWrite(IN_4B, LOW);
  isMovingForward = false;
  isTurningLeft = false;
  isTurningRight = false;
  isMovingBackward = false;
  isRotatingLeft = false;
  isRotatingRight = false;
  isSwipeLeft = false;
  isSwipeRight = false;
}

void geserkanan(){
  analogWrite(IN_1D, fast); //depan kiri mundur
  analogWrite(IN_2D, 0);
  analogWrite(IN_3D, 0);    //depan kanan maju
  analogWrite(IN_4D, fast);
  analogWrite(IN_1B, fast); //bel kanan mundur
  analogWrite(IN_2B, 0);
  analogWrite(IN_3B, fast); //bel kiri maju
  analogWrite(IN_4B, 0);
  isMovingForward = false;
  isTurningLeft = false;
  isTurningRight = false;
  isMovingBackward = false;
  isRotatingLeft = false;
  isRotatingRight = false;
  isSwipeLeft = true;
  isSwipeRight = false;
}

void geserkiri(){
  analogWrite(IN_1D, 0);     //depan kiri maju
  analogWrite(IN_2D, fast);
  analogWrite(IN_3D, fast);  //depan kanan mundur
  analogWrite(IN_4D, 0);
  analogWrite(IN_1B, 0);     //bel kanan maju
  analogWrite(IN_2B, fast);
  analogWrite(IN_3B, 0);     //bel kiri mundur
  analogWrite(IN_4B, fast);
  isMovingForward = false;
  isTurningLeft = false;
  isTurningRight = false;
  isMovingBackward = false;
  isRotatingLeft = false;
  isRotatingRight = false;
  isSwipeLeft = false;
  isSwipeRight = true;
}

void putarkiri(){
  analogWrite(IN_1D, fast);  //depan kiri mundur
  analogWrite(IN_2D, 0);
  analogWrite(IN_3D, 0);     //depan kanan maju
  analogWrite(IN_4D, fast);
  analogWrite(IN_1B, 0);     //bel kanan maju
  analogWrite(IN_2B, fast);
  analogWrite(IN_3B, 0);     //bel kiri mundur
  analogWrite(IN_4B, fast);
  isMovingForward = false;
  isTurningLeft = false;
  isTurningRight = false;
  isMovingBackward = false;
  isRotatingLeft = true;
  isRotatingRight = false;
  isSwipeLeft = false;
  isSwipeRight = false;

}

void putarkanan(){
  analogWrite(IN_1D, 0);     //depan kiri maju
  analogWrite(IN_2D, fast);
  analogWrite(IN_3D, fast);  //depan kanan mundur
  analogWrite(IN_4D, 0);
  analogWrite(IN_1B, fast);  //bel kanan mundur
  analogWrite(IN_2B, 0);
  analogWrite(IN_3B, fast);  //bel kiri maju
  analogWrite(IN_4B, 0);
  isMovingForward = false;
  isTurningLeft = false;
  isTurningRight = false;
  isMovingBackward = false;
  isRotatingLeft = false;
  isRotatingRight = true;
  isSwipeLeft = false;
  isSwipeRight = false;
}

void belokkiri(){
  analogWrite(IN_1D, 0);
  analogWrite(IN_2D, 0);
  analogWrite(IN_3D, 0);     //depan kanan maju
  analogWrite(IN_4D, fast);
  analogWrite(IN_1B, 0);     //bel kanan maju
  analogWrite(IN_2B, fast);
  analogWrite(IN_3B, 0);
  analogWrite(IN_4B, 0);
  isMovingForward = false;
  isTurningLeft = true;
  isTurningRight = false;
  isMovingBackward = false;
  isRotatingLeft = false;
  isRotatingRight = false;
  isSwipeLeft = false;
  isSwipeRight = false;
}

void belokkanan(){
  analogWrite(IN_1D, 0);    //depan kiri maju
  analogWrite(IN_2D, fast);
  analogWrite(IN_3D, 0); 
  analogWrite(IN_4D, 0);
  analogWrite(IN_1B, 0);
  analogWrite(IN_2B, 0);
  analogWrite(IN_3B, fast); //bel kiri maju
  analogWrite(IN_4B, 0);
  isMovingForward = false;
  isTurningLeft = false;
  isTurningRight = true;
  isMovingBackward = false;
  isRotatingLeft = false;
  isRotatingRight = false;
  isSwipeLeft = false;
  isSwipeRight = false;
}