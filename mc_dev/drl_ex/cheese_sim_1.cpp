

//参照：遷移先の状態
double vending_machine(int s, int a, int &sd){
    double reward;
    if (a==0){ // 電源ボタンを押したとき
        sd = !s; // 電源を反転
        reward = 0;
    }
    else{      // Cheeボタンを押したとき
        if (s==1){ // When sd is on
            sd = s;
            reward = 10;
        }
        else{      // When sd is off
            sd = s
