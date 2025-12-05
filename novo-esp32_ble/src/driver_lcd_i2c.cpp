#include "driver_lcd_i2c.hpp"
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Construtor
DriverLCD_I2C::DriverLCD_I2C(uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows)
  :_lcdAddr(lcd_Addr), _lcdCols(lcd_cols), _lcdRows(lcd_rows) {
    // Inicializa as variáveis de posição do cursor
    _cursorCol = 0;  // Coluna inicial do cursor
    _cursorRow = 0;  // Linha inicial do cursor
}

// Destruidor
DriverLCD_I2C::~DriverLCD_I2C() {
}

// Getter para a coluna do cursor
uint8_t DriverLCD_I2C::getCursorCol() const {
    return _cursorCol;  // Retorna a coluna atual do cursor
}

// Getter para a linha do cursor
uint8_t DriverLCD_I2C::getCursorRow() const {
    return _cursorRow;  // Retorna a linha atual do cursor
}

// Setter para a coluna do cursor
void DriverLCD_I2C::setCursorCol(uint8_t col) {
    if (col < _lcdCols) {  // Verifica se a coluna está dentro dos limites
        _cursorCol = col;  // Atualiza a coluna
    }
}

// Setter para a linha do cursor
void DriverLCD_I2C::setCursorRow(uint8_t row) {
    if (row < _lcdRows) {  // Verifica se a linha está dentro dos limites
        _cursorRow = row;  // Atualiza a linha
    }
}

// Método para exibir o ângulo no LCD
void DriverLCD_I2C::postAngle(float angle, float ctAngle) {
    clear();  // Limpa a tela antes de imprimir

    // Formata os valores para 2 casas decimais
    char angleStr[10], ctAngleStr[10];
    sprintf(angleStr, "%.1f", angle);
    sprintf(ctAngleStr, "%.1f", ctAngle);

    // Monta a mensagem formatada
    String line = "c.a:" + String(angleStr) + "  d.a:" + String(ctAngleStr);

    // Garante que a linha não ultrapasse o tamanho do LCD
    if (line.length() > _lcdCols) {
        line = line.substring(0, _lcdCols);  // Trunca a mensagem se necessário
    }
    lcd.setCursor((_lcdCols - line.length()) / 2, 0);

    // Posiciona no início da tela
    //lcd.setCursor(0, 0);
    lcd.print(line);

    // Atualiza os valores internos do cursor
    _cursorRow = 1;  // Move para a próxima linha
    _cursorCol = 0;
    lcd.setCursor(_cursorCol, _cursorRow);
}

void DriverLCD_I2C::postTofRight(float distance) {
    if(_cursorRow > 3){
        _cursorRow = 0;
        _cursorCol = 0;
        clear();
    }
    // Cria a string "TOF Right: " mais o valor da distância
    char distanceStr[10];  // Buffer para armazenar a string convertida

    // Usando sprintf para formatar o float em string com 2 casas decimais
    sprintf(distanceStr, "%.4f", distance);  // Converte o valor float para string com 2 casas decimais

    String fullMessage = String("TOF Right: ") + distanceStr;  // Junta "TOF Right: " com o valor da distância

    // Verifica se a mensagem cabe na linha
    if (fullMessage.length() > (_lcdCols - _cursorCol)) {
	_cursorRow = 0;
        _cursorCol = 0;
        clear();
        // Caso não caiba, centraliza "TOF Right:" na primeira linha
        int tofMessageLength = 11;  // "TOF Right:" tem 11 caracteres
        int distanceStrLength = strlen(distanceStr);
        
        // Centraliza "TOF Right:"
        lcd.setCursor((_lcdCols - tofMessageLength) / 2, 0);  // Calcula a posição para centralizar "TOF Right:"
        lcd.print("TOF Right:");
        _cursorRow++;
        // Centraliza o valor da distância na segunda linha
        lcd.setCursor((_lcdCols - distanceStrLength) / 2, 1);  // Centraliza o valor da distância
        lcd.print(distanceStr);  // Exibe o valor da distância
    } else {
        clearLine(_cursorRow);
	// Caso a mensagem caiba na linha, imprime tudo na mesma linha
        lcd.setCursor(_cursorCol, _cursorRow);  // Posiciona o cursor
        lcd.print("TOF Right: ");
        lcd.print(distanceStr);  // Exibe o valor da distância
    }
    _cursorCol = 0;
    _cursorRow++;
    lcd.setCursor(_cursorCol, _cursorRow);  // Atualiza o cursor no LCD
}

void DriverLCD_I2C::postTofLeft(float distance) {
    if(_cursorRow > 3){
        _cursorRow = 0;
        _cursorCol = 0;
        clear();
    }
    // Cria a string "TOF Left: " mais o valor da distancia
    char distanceStr[10];  // Buffer para armazenar a string convertida

    // Usando sprintf para formatar o float em string com 2 casas decimais
    sprintf(distanceStr, "%.4f", distance);  // Converte o valor float para string com 2 casas decimais

    String fullMessage = String("TOF Left: ") + distanceStr;  // Junta "TOF Left: " com o valor da distancia

    // Verifica se a mensagem cabe na linha
    if (fullMessage.length() > (_lcdCols - _cursorCol)) {
	clear();  // Limpa a tela antes de imprimir
        // Caso não caiba, centraliza "TOF Left:" na primeira linha
        int tofMessageLength = 10;  // "TOF Left:" tem 10 caracteres
        int distanceStrLength = strlen(distanceStr);
        // Centraliza "TOF Left:"
        lcd.setCursor((_lcdCols - tofMessageLength) / 2, _cursorRow);  // Calcula a posição para centralizar "TOF Left:"
        lcd.print("TOF Left:");
        _cursorRow++;
        // Centraliza o valor da distância na segunda linha
        lcd.setCursor((_lcdCols - distanceStrLength) / 2, _cursorRow);  // Centraliza o valor da distância
        lcd.print(distanceStr);  // Exibe o valor da distância
    } else {
        clearLine(_cursorRow);
	// Caso a mensagem caiba na linha, imprime tudo na mesma linha
        lcd.setCursor(_cursorCol, _cursorRow);  // Posiciona o cursor
        lcd.print("TOF Left: ");
        lcd.print(distanceStr);  // Exibe o valor da distância
    }
    _cursorCol = 0;
    _cursorRow++;
    lcd.setCursor(_cursorCol, _cursorRow);  // Atualiza o cursor no LCD
}

void DriverLCD_I2C::postDuty(int duty_l, int duty_r) {

    // Centraliza "setDutycycle()" na primeira linha
    String title = "setDutycycle()";//14
    int titlePos = (_lcdCols - title.length()) / 2;
    lcd.setCursor(titlePos, 1);
    lcd.print(title);

    // Formata e imprime "duty_L = valor%" na segunda linha
    lcd.setCursor(0, 2);
    lcd.print("   duty_L = ");
    lcd.print(duty_l);
    lcd.print("%");

    // Formata e imprime "duty_R = valor%" na terceira linha
    lcd.setCursor(0, 3);
    lcd.print("   duty_R = ");
    lcd.print(duty_r);
    lcd.print("%");
}

void DriverLCD_I2C::postAccelerometer(float accelX, float accelY, float accelZ) {
    // Cria as strings para cada valor de aceleração com 6 casas decimais
    char accelXStr[10], accelYStr[10], accelZStr[10];  // Buffers para armazenar as strings convertidas

    // Usando sprintf para formatar os valores float em strings com 6 casas decimais
    sprintf(accelXStr, "%.3f", accelX);  // Converte o valor de accelX para string com 6 casas decimais
    sprintf(accelYStr, "%.3f", accelY);  // Converte o valor de accelY para string com 6 casas decimais
    sprintf(accelZStr, "%.3f", accelZ);  // Converte o valor de accelZ para string com 6 casas decimais

    // Exibe os valores das acelerações nas linhas correspondentes
    lcd.setCursor(_cursorCol, _cursorRow);  // Posiciona o cursor para a primeira linha
    lcd.print("  Accel_x: ");
    lcd.print(accelXStr);  // Exibe o valor de aceleração em x
    lcd.print(" g");
    _cursorRow++;  // Avança para a próxima linha

    lcd.setCursor(_cursorCol, _cursorRow);  // Posiciona o cursor para a segunda linha
    lcd.print("  Accel_y: ");
    lcd.print(accelYStr);  // Exibe o valor de aceleração em y
    lcd.print(" g");
    _cursorRow++;  // Avança para a próxima linha

    lcd.setCursor(_cursorCol, _cursorRow);  // Posiciona o cursor para a terceira linha
    lcd.print("  Accel_z: ");
    lcd.print(accelZStr);  // Exibe o valor de aceleração em z
    lcd.print(" g");
    _cursorRow++;  // Avança para a próxima linha
}


void DriverLCD_I2C::postSetPoint(float angle) {
    if(_cursorRow > 3){
        _cursorRow = 0;
        _cursorCol = 0;
        clear();
    }
    // Cria a string "SetPoint: " mais o valor da angulação
    char setPointStr[10];  // Buffer para armazenar a string convertida

    // Usando sprintf para formatar o float em string com 2 casas decimais
    sprintf(setPointStr, "%.2f", angle);  // Converte o valor float para string com 2 casas decimais

    String fullMessage = String("SetPoint: ") + setPointStr;  // Junta "SetPoint: " com o valor da angulação

    // Verifica se a mensagem cabe na linha
    if (fullMessage.length() > (_lcdCols - _cursorCol)) {
	clear();  // Limpa a tela antes de imprimir
        // Caso não caiba, centraliza "SetPoint:" na primeira linha
        int tofMessageLength = 10;  // "SetPoint:" tem 10 caracteres
        int setPointStrLength = strlen(setPointStr);
        // Centraliza "SetPoint:"
        lcd.setCursor((_lcdCols - tofMessageLength) / 2, _cursorRow);  // Calcula a posição para centralizar "SetPoint:"
        lcd.print("SetPoint:");
        _cursorRow++;
        // Centraliza o valor da angulação na segunda linha
        lcd.setCursor((_lcdCols - setPointStrLength) / 2, _cursorRow);  // Centraliza o valor da angulação
        lcd.print(setPointStr);  // Exibe o valor da angulação
        // Atualiza o cursor interno
    } else {
        clearLine(_cursorRow);
	// Caso a mensagem caiba na linha, imprime tudo na mesma linha
        lcd.setCursor(_cursorCol, _cursorRow);  // Posiciona o cursor
        lcd.print("SetPoint: ");
        lcd.print(setPointStr);  // Exibe o valor da angulação
    }
    _cursorCol = 0;
    _cursorRow++;
    lcd.setCursor(_cursorCol, _cursorRow);  // Atualiza o cursor no LCD
}

void DriverLCD_I2C::postOperatingMode(int modo) {
    if (_cursorRow > 1) {
        _cursorRow = 0;
        _cursorCol = 0;
        clear();
    }

    // Declara as strings para os modos
    const char* modeText = nullptr;

    // Determina a mensagem a ser exibida com base no modo
    switch (modo) {
        case 0:
            modeText = "MANUAL MODE";
            break;
        case 1:
            modeText = "AUTOMATIC MODE";
            break;
        case 2:
            modeText = "REHEARSAL MODE";
            break;
    }

    _cursorRow++;
    // Centraliza e imprime a mensagem na primeira linha
    lcd.setCursor((_lcdCols - strlen(modeText)) / 2, _cursorRow);
    lcd.print(modeText);

    _cursorCol = 0;
    _cursorRow++;
    lcd.setCursor(_cursorCol, _cursorRow);  // Atualiza o cursor no LCD
}

void DriverLCD_I2C::postConsumption(float voltage, float current, const char* voltageUnit, const char* currentUnit) {
    // Garante que o cursor está na posição inicial e limpa a tela
    // _cursorRow = 0;
    // _cursorCol = 0;
    // clear();

    // Linha 0: "CONSUMPTION" centralizado
    const char* title = "CONSUMPTION:";
    lcd.setCursor((_lcdCols - strlen(title)) / 2, _cursorRow);
    lcd.print(title);
    _cursorRow++;

    // Linha 1: Corrente com 4 casas decimais e unidade
    char currentStr[15];
    sprintf(currentStr, "%.6f %s", current, currentUnit);  // Formata corrente com 4 casas decimais + unidade

    lcd.setCursor((_lcdCols - strlen(currentStr)) / 2, _cursorRow);
    lcd.print(currentStr);
    _cursorRow++;

    // Linha 2: Tensão com 4 casas decimais e unidade
    char voltageStr[15];
    sprintf(voltageStr, "%.6f %s", voltage, voltageUnit);  // Formata tensão com 4 casas decimais + unidade

    lcd.setCursor((_lcdCols - strlen(voltageStr)) / 2, _cursorRow);
    lcd.print(voltageStr);
    _cursorRow++;
    
    // Posiciona o cursor na última linha, pronto para uma nova exibição no futuro
    _cursorCol = 0;
    lcd.setCursor(_cursorCol, _cursorRow);
}


void DriverLCD_I2C::dashboardLightOff(){
    lcd.init();    // Inicializa o LCD
    lcd.noBacklight();
}

void DriverLCD_I2C::dashboardLightOn(){
    lcd.init();    // Inicializa o LCD
    lcd.backlight();
}

void DriverLCD_I2C::clear(){
    lcd.clear();
    _cursorRow = 0;
    _cursorCol = 0;
}
void DriverLCD_I2C::clearLine(uint8_t line) {
    // Verifica se a linha está dentro do limite do LCD
    if (line >= _lcdRows) return;

    // Move o cursor para o início da linha desejada
    lcd.setCursor(0, line);
    _cursorRow = line;
    _cursorCol = 0;
    // Sobrescreve a linha com espaços
    for (uint8_t i = 0; i < _lcdCols; i++) {
        lcd.print(" ");
    }

    // atualiza o cursor
    lcd.setCursor(_cursorCol , _cursorRow);
}