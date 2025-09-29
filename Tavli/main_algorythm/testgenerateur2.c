#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

void initialisation_tavli(int tavli[4][13]){
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 13; j++){
            tavli[i][j] = 0; 
        }
    }
    // 1 pour blanc
    // 2 pour rouge
    //Première ligne
    tavli[0][0] = 1; 
    tavli[1][0] = 5; 
    tavli[0][4] = 2; 
    tavli[1][4] = 3; 
    tavli[0][6] = 2; 
    tavli[1][6] = 5; 
    tavli[0][11] = 1; 
    tavli[1][11] = 2; 
    //deuxième ligne
    tavli[2][0] = 2;
    tavli[3][0] = 5;
    tavli[2][4] = 1;
    tavli[3][4] = 3;
    tavli[2][6] = 1;
    tavli[3][6] = 5;
    tavli[2][11] = 2;
    tavli[3][11] = 2;
}

int generator_dice(int lower, int upper){
    int num = (rand() % (upper - lower + 1)) + lower;  
    return num; 
}

int max(int a, int b) {
    return (a > b) ? a : b;
}

int generator_wich_dice_to_choose(int lower, int upper){
    int num = (rand() % (upper - lower + 1)) + lower;  
    return num; 
}

void picture_generator(int tavli[4][13]){
    int a = tavli[1][0]; 
    int b = tavli[1][1]; 
    int c = tavli[1][2]; 
    int d = tavli[1][3]; 
    int e = tavli[1][4]; 
    int f = tavli[1][5]; 
    int g = tavli[1][6]; 
    int h = tavli[1][7]; 
    int i = tavli[1][8]; 
    int j = tavli[1][9]; 
    int k = tavli[1][10]; 
    int l = tavli[1][11]; 
    int m = tavli[1][12];

    int a2 = 0; 
    int b2 = 0; 
    int c2 = 0; 
    int d2 = 0; 
    int e2 = 0; 
    int f2 = 0; 
    int g2 = 0; 
    int h2 = 0; 
    int i2 = 0; 
    int j2 = 0; 
    int k2 = 0; 
    int l2 = 0; 
    int m2 = 0; 

    int maximum = max(max(max(max(max(max(max(max(max(max(max(max(a, b), c), d), e), f), g), h), i), j), k), l), m);

    for (int w = 0; w < maximum; w++){
        if((a2 < a) && (tavli[0][0] == 1) ){
            printf("O"); 
        }
        if((a2 < a) && (tavli[0][0] == 2) ){
            printf("X"); 
        }
        if(tavli[0][0] == 0){
            printf("|"); 
        }
        if((a2 >= a)&&(tavli[0][0] == 2)){
            printf("|"); 
        }
        if((a2 >= a)&&(tavli[0][0] == 1)){
            printf("|"); 
        }
        a2++; 

        if((b2 < b) && (tavli[0][1] == 1) ){
            printf("O"); 
        }
        if((b2 < b) && (tavli[0][1] == 2) ){
            printf("X"); 
        }
        if(tavli[0][1] == 0){
            printf("|"); 
        }
        if((b2 >= b)&&(tavli[0][1] == 2)){
            printf("|"); 
        }
        if((b2 >= b)&&(tavli[0][1] == 1)){
            printf("|"); 
        }
        b2++;

        if((c2 < c) && (tavli[0][2] == 1) ){
            printf("O"); 
        }
        if((c2 < c) && (tavli[0][2] == 2) ){
            printf("X"); 
        }
        if(tavli[0][2] == 0){
            printf("|"); 
        }
        if((c2 >= c)&&(tavli[0][2] == 2)){
            printf("|"); 
        }
        if((c2 >= c)&&(tavli[0][2] == 1)){
            printf("|"); 
        }
        c2++;

        if((d2 < d) && (tavli[0][3] == 1) ){
            printf("O"); 
        }
        if((d2 < d) && (tavli[0][3] == 2) ){
            printf("X"); 
        }
        if(tavli[0][3] == 0){
            printf("|"); 
        }
        if((d2 >= d)&&(tavli[0][3] == 2)){
            printf("|"); 
        }
        if((d2 >= d)&&(tavli[0][3] == 1)){
            printf("|"); 
        }
        d2++;

        if((e2 < e) && (tavli[0][4] == 1) ){
            printf("O"); 
        }
        if((e2 < e) && (tavli[0][4] == 2) ){
            printf("X"); 
        }
        if(tavli[0][4] == 0){
            printf("|"); 
        }
        if((e2 >= e)&&(tavli[0][4] == 2)){
            printf("|"); 
        }
        if((e2 >= e)&&(tavli[0][4] == 1)){
            printf("|"); 
        }
        e2++;

        if((f2 < f) && (tavli[0][5] == 1) ){
            printf("O"); 
        }
        if((f2 < f) && (tavli[0][5] == 2) ){
            printf("X"); 
        }
        if(tavli[0][5] == 0){
            printf("|"); 
        }
        if((f2 >= f)&&(tavli[0][5] == 2)){
            printf("|"); 
        }
        if((f2 >= f)&&(tavli[0][5] == 1)){
            printf("|"); 
        }
        f2++;

        if((g2 < g) && (tavli[0][6] == 1) ){
            printf("O"); 
        }
        if((g2 < g) && (tavli[0][6] == 2) ){
            printf("X"); 
        }
        if(tavli[0][6] == 0){
            printf("|"); 
        }
        if((g2 >= g)&&(tavli[0][6] == 2)){
            printf("|"); 
        }
        if((g2 >= g)&&(tavli[0][6] == 1)){
            printf("|"); 
        }
        g2++;

        if((h2 < h) && (tavli[0][7] == 1) ){
            printf("O"); 
        }
        if((h2 < h) && (tavli[0][7] == 2) ){
            printf("X"); 
        }
        if(tavli[0][7] == 0){
            printf("|"); 
        }
        if((h2 >= h)&&(tavli[0][7] == 2)){
            printf("|"); 
        }
        if((h2 >= h)&&(tavli[0][7] == 1)){
            printf("|"); 
        }
        h2++;

        if((i2 < i) && (tavli[0][8] == 1) ){
            printf("O"); 
        }
        if((i2 < i) && (tavli[0][8] == 2) ){
            printf("X"); 
        }
        if(tavli[0][8] == 0){
            printf("|"); 
        }
        if((i2 >= i)&&(tavli[0][8] == 2)){
            printf("|"); 
        }
        if((i2 >= i)&&(tavli[0][8] == 1)){
            printf("|"); 
        }
        i2++;

        if((j2 < j) && (tavli[0][9] == 1) ){
            printf("O"); 
        }
        if((j2 < j) && (tavli[0][9] == 2) ){
            printf("X"); 
        }
        if(tavli[0][9] == 0){
            printf("|"); 
        }
        if((j2 >= j)&&(tavli[0][9] == 2)){
            printf("|"); 
        }
        if((j2 >= j)&&(tavli[0][9] == 1)){
            printf("|"); 
        }
        j2++;

        if((k2 < k) && (tavli[0][10] == 1) ){
            printf("O"); 
        }
        if((k2 < k) && (tavli[0][10] == 2) ){
            printf("X"); 
        }
        if(tavli[0][10] == 0){
            printf("|"); 
        }
        if((k2 >= k)&&(tavli[0][10] == 2)){
            printf("|"); 
        }
        if((k2 >= k)&&(tavli[0][10] == 1)){
            printf("|"); 
        }
        k2++;

        if((l2 < l) && (tavli[0][11] == 1) ){
            printf("O"); 
        }
        if((l2 < l) && (tavli[0][11] == 2) ){
            printf("X"); 
            }
        if(tavli[0][11] == 0){
            printf("|"); 
        }
        if((l2 >= l)&&(tavli[0][11] == 2)){
            printf("|"); 
        }
        if((l2 >= l)&&(tavli[0][11] == 1)){
            printf("|"); 
        }
        l2++;

        if((m2 < m) && (tavli[0][12] == 1) ){
            printf("O"); 
        }
        if((m2 < m) && (tavli[0][12] == 2) ){
            printf("X"); 
        }
        if(tavli[0][12] == 0){
            printf("|"); 
        }
        if((m2 >= m)&&(tavli[0][12] == 2)){
            printf("|"); 
        }
        if((m2 >= m)&&(tavli[0][12] == 1)){
            printf("|"); 
        }
        m2++;

        printf("\n"); 

    }
    printf("_____________\n"); 
    printf("\n"); 


    a = tavli[3][0]; 
    b = tavli[3][1]; 
    c = tavli[3][2]; 
    d = tavli[3][3]; 
    e = tavli[3][4]; 
    f = tavli[3][5]; 
    g = tavli[3][6]; 
    h = tavli[3][7]; 
    i = tavli[3][8]; 
    j = tavli[3][9]; 
    k = tavli[3][10]; 
    l = tavli[3][11]; 
    m = tavli[3][12];

    a2 = 0; 
    b2 = 0; 
    c2 = 0; 
    d2 = 0; 
    e2 = 0; 
    f2 = 0; 
    g2 = 0; 
    h2 = 0; 
    i2 = 0; 
    j2 = 0; 
    k2 = 0; 
    l2 = 0; 
    m2 = 0; 

    maximum = max(max(max(max(max(max(max(max(max(max(max(max(a, b), c), d), e), f), g), h), i), j), k), l), m);


    for (int w = 0; w < maximum; w++){
        if((a2 < a) && (tavli[2][0] == 1) ){
            printf("O"); 
        }
        if((a2 < a) && (tavli[2][0] == 2) ){
            printf("X"); 
        }
        if(tavli[2][0] == 0){
            printf("|"); 
        }
        if((a2 >= a)&&(tavli[2][0] == 2)){
            printf("|"); 
        }
        if((a2 >= a)&&(tavli[2][0] == 1)){
            printf("|"); 
        }
        a2++; 

        if((b2 < b) && (tavli[2][1] == 1) ){
            printf("O"); 
        }
        if((b2 < b) && (tavli[2][1] == 2) ){
            printf("X"); 
        }
        if(tavli[2][1] == 0){
            printf("|"); 
        }
        if((b2 >= b)&&(tavli[2][1] == 2)){
            printf("|"); 
        }
        if((b2 >= b)&&(tavli[2][1] == 1)){
            printf("|"); 
        }
        b2++;

        if((c2 < c) && (tavli[2][2] == 1) ){
            printf("O"); 
        }
        if((c2 < c) && (tavli[2][2] == 2) ){
            printf("X"); 
        }
        if(tavli[2][2] == 0){
            printf("|"); 
        }
        if((c2 >= c)&&(tavli[2][2] == 2)){
            printf("|"); 
        }
        if((c2 >= c)&&(tavli[2][2] == 1)){
            printf("|"); 
        }
        c2++;

        if((d2 < d) && (tavli[2][3] == 1) ){
            printf("O"); 
        }
        if((d2 < d) && (tavli[2][3] == 2) ){
            printf("X"); 
        }
        if(tavli[2][3] == 0){
            printf("|"); 
        }
        if((d2 >= d)&&(tavli[2][3] == 2)){
            printf("|"); 
        }
        if((d2 >= d)&&(tavli[2][3] == 1)){
            printf("|"); 
        }
        d2++;

        if((e2 < e) && (tavli[2][4] == 1) ){
            printf("O"); 
        }
        if((e2 < e) && (tavli[2][4] == 2) ){
            printf("X"); 
        }
        if(tavli[2][4] == 0){
            printf("|"); 
        }
        if((e2 >= e)&&(tavli[2][4] == 2)){
            printf("|"); 
        }
        if((e2 >= e)&&(tavli[2][4] == 1)){
            printf("|"); 
        }
        e2++;

        if((f2 < f) && (tavli[2][5] == 1) ){
            printf("O"); 
        }
        if((f2 < f) && (tavli[2][5] == 2) ){
            printf("X"); 
        }
        if(tavli[2][5] == 0){
            printf("|"); 
        }
        if((f2 >= f)&&(tavli[2][5] == 2)){
            printf("|"); 
        }
        if((f2 >= f)&&(tavli[2][5] == 1)){
            printf("|"); 
        }
        f2++;

        if((g2 < g) && (tavli[2][6] == 1) ){
            printf("O"); 
        }
        if((g2 < g) && (tavli[2][6] == 2) ){
            printf("X"); 
        }
        if(tavli[2][6] == 0){
            printf("|"); 
        }
        if((g2 >= g)&&(tavli[2][6] == 2)){
            printf("|"); 
        }
        if((g2 >= g)&&(tavli[2][6] == 1)){
            printf("|"); 
        }
        g2++;

        if((h2 < h) && (tavli[2][7] == 1) ){
            printf("O"); 
        }
        if((h2 < h) && (tavli[2][7] == 2) ){
            printf("X"); 
        }
        if(tavli[2][7] == 0){
            printf("|"); 
        }
        if((h2 >= h)&&(tavli[2][7] == 2)){
            printf("|"); 
        }
        if((h2 >= h)&&(tavli[2][7] == 1)){
            printf("|"); 
        }
        h2++;

        if((i2 < i) && (tavli[2][8] == 1) ){
            printf("O"); 
        }
        if((i2 < i) && (tavli[2][8] == 2) ){
            printf("X"); 
        }
        if(tavli[2][8] == 0){
            printf("|"); 
        }
        if((i2 >= i)&&(tavli[2][8] == 2)){
            printf("|"); 
        }
        if((i2 >= i)&&(tavli[2][8] == 1)){
            printf("|"); 
        }
        i2++;

        if((j2 < j) && (tavli[2][9] == 1) ){
            printf("O"); 
        }
        if((j2 < j) && (tavli[2][9] == 2) ){
            printf("X"); 
        }
        if(tavli[2][9] == 0){
            printf("|"); 
        }
        if((j2 >= j)&&(tavli[2][9] == 2)){
            printf("|"); 
        }
        if((j2 >= j)&&(tavli[2][9] == 1)){
            printf("|"); 
        }
        j2++;

        if((k2 < k) && (tavli[2][10] == 1) ){
            printf("O"); 
        }
        if((k2 < k) && (tavli[2][10] == 2) ){
            printf("X"); 
        }
        if(tavli[2][10] == 0){
            printf("|"); 
        }
        if((k2 >= k)&&(tavli[2][10] == 2)){
            printf("|"); 
        }
        if((k2 >= k)&&(tavli[2][10] == 1)){
            printf("|"); 
        }
        k2++;

        if((l2 < l) && (tavli[2][11] == 1) ){
            printf("O"); 
        }
        if((l2 < l) && (tavli[2][11] == 2) ){
            printf("X"); 
            }
        if(tavli[2][11] == 0){
            printf("|"); 
        }
        if((l2 >= l)&&(tavli[2][11] == 2)){
            printf("|"); 
        }
        if((l2 >= l)&&(tavli[2][11] == 1)){
            printf("|"); 
        }
        l2++;

        if((m2 < m) && (tavli[2][12] == 1) ){
            printf("O"); 
        }
        if((m2 < m) && (tavli[2][12] == 2) ){
            printf("X"); 
        }
        if(tavli[2][12] == 0){
            printf("|"); 
        }
        if((m2 >= m)&&(tavli[2][12] == 2)){
            printf("|"); 
        }
        if((m2 >= m)&&(tavli[2][12] == 1)){
            printf("|"); 
        }
        m2++;

        printf("\n"); 

    }

    printf("_____________\n"); 
    printf("_____________\n"); 


    printf("\n"); 

}

void position_changer(int array_to_change[4][13], int couleur, int position_intial_ligne,int position_intial_colone , int nombre_dé){
    //printf("position ligne : %d\n", position_intial_ligne); 
    //printf("position colone : %d\n", position_intial_colone); 
    //printf("nombre dé utilisé = %d\n", nombre_dé); 
    
    if(couleur == 1){
        //printf("fonctioncouleur=1\n"); 
        if(position_intial_ligne == 0){
            //printf("ici1\n"); 
            if(position_intial_colone >= nombre_dé){
                //printf("ici2\n"); 

                if((array_to_change[0][position_intial_colone - nombre_dé] == 2)&&(array_to_change[1][position_intial_colone - nombre_dé] == 1)){
                    //printf("ici3\n"); 
                    array_to_change[0][position_intial_colone - nombre_dé] = 1;
                    array_to_change[1][position_intial_colone - nombre_dé] = 0;
                    array_to_change[2][12] = 2;  
                    array_to_change[3][12] = array_to_change[3][12] + 1;  

                    array_to_change[1][position_intial_colone] = array_to_change[1][position_intial_colone] - 1; 
                    array_to_change[1][position_intial_colone - nombre_dé] = array_to_change[1][position_intial_colone - nombre_dé] + 1; 
                }
                else if(array_to_change[0][position_intial_colone - nombre_dé] == 0){
                    //printf("ici4\n"); 
                    array_to_change[0][position_intial_colone - nombre_dé] = 1; 
                    array_to_change[1][position_intial_colone] = array_to_change[1][position_intial_colone] - 1; 
                    array_to_change[1][position_intial_colone - nombre_dé] = 1;  
                }
                else if(array_to_change[0][position_intial_colone - nombre_dé] == 1){
                    //printf("ici5\n"); 
                    array_to_change[1][position_intial_colone] = array_to_change[1][position_intial_colone] - 1; 
                    array_to_change[1][position_intial_colone - nombre_dé] = array_to_change[1][position_intial_colone - nombre_dé] + 1; 
                }
                if(array_to_change[1][position_intial_colone] == 0){
                    //printf("ici6\n"); 
                    array_to_change[0][position_intial_colone] = 0; 
                }
            }
            else{
                //printf("ici7\n"); 
                nombre_dé = nombre_dé - position_intial_colone - 1; 
                int position_intial_colone_2 = 0; 

                if((array_to_change[2][position_intial_colone_2 + nombre_dé] == 2)&&(array_to_change[3][position_intial_colone_2 + nombre_dé] == 1)){
                    //printf("ici8\n"); 
                    array_to_change[2][position_intial_colone_2 + nombre_dé] = 1;
                    array_to_change[3][position_intial_colone_2 + nombre_dé] = 0;
                    array_to_change[2][12] = 2;  
                    array_to_change[3][12] = array_to_change[3][12] + 1;  

                    array_to_change[1][position_intial_colone] = array_to_change[1][position_intial_colone] - 1; 
                    array_to_change[3][position_intial_colone_2 + nombre_dé] = array_to_change[3][position_intial_colone_2 + nombre_dé] + 1; 
                }
                else if(array_to_change[2][position_intial_colone_2 + nombre_dé] == 0){
                    //printf("ici9\n"); 
                    array_to_change[2][position_intial_colone_2 + nombre_dé] = 1; 
                    array_to_change[1][position_intial_colone] = array_to_change[1][position_intial_colone] - 1; 
                    array_to_change[3][position_intial_colone_2 + nombre_dé] = 1;  
                }
                else if(array_to_change[2][position_intial_colone_2 + nombre_dé] == 1){
                    //printf("ici10\n"); 
                    array_to_change[1][position_intial_colone] = array_to_change[1][position_intial_colone] - 1; 
                    array_to_change[3][position_intial_colone_2 + nombre_dé] = array_to_change[3][position_intial_colone_2 + nombre_dé] + 1; 
                }
                if(array_to_change[1][position_intial_colone] == 0){
                    //printf("ici11\n"); 
                    array_to_change[0][position_intial_colone] = 0; 
                }
                
            }
        }

        else if(position_intial_ligne == 1){
            //printf("ici12\n"); 
                if((array_to_change[2][position_intial_colone + nombre_dé] == 2)&&(array_to_change[3][position_intial_colone + nombre_dé] == 1)){
                    //printf("ici13\n"); 
                    array_to_change[2][position_intial_colone + nombre_dé] = 1;
                    array_to_change[3][position_intial_colone + nombre_dé] = 0;
                    array_to_change[2][12] = 2;  
                    array_to_change[3][12] = array_to_change[3][12] + 1;  

                    array_to_change[3][position_intial_colone] = array_to_change[3][position_intial_colone] - 1; 
                    array_to_change[3][position_intial_colone + nombre_dé] = array_to_change[3][position_intial_colone + nombre_dé] + 1; 
                }
                else if(array_to_change[2][position_intial_colone + nombre_dé] == 0){
                    //printf("ici14\n"); 
                    array_to_change[2][position_intial_colone + nombre_dé] = 1; 
                    array_to_change[3][position_intial_colone] = array_to_change[3][position_intial_colone] - 1; 
                    array_to_change[3][position_intial_colone + nombre_dé] = 1;  
                }
                else if(array_to_change[2][position_intial_colone + nombre_dé] == 1){
                    //printf("ici15\n"); 
                    array_to_change[3][position_intial_colone] = array_to_change[3][position_intial_colone] - 1; 
                    array_to_change[3][position_intial_colone + nombre_dé] = array_to_change[3][position_intial_colone + nombre_dé] + 1; 
                }
                if(array_to_change[3][position_intial_colone] == 0){
                    //printf("ici16\n"); 
                    array_to_change[2][position_intial_colone] = 0; 
                }
        }

    }

    else if (couleur == 2){
        //printf("fonctioncouleur=2\n"); 
        if(position_intial_ligne == 1){
            if(position_intial_colone >= nombre_dé){
                //printf("la1\n"); 

                if((array_to_change[2][position_intial_colone - nombre_dé] == 1)&&(array_to_change[3][position_intial_colone - nombre_dé] == 1)){
                    //printf("la2\n"); 
                    array_to_change[2][position_intial_colone - nombre_dé] = 2;
                    array_to_change[3][position_intial_colone - nombre_dé] = 0;
                    array_to_change[0][12] = 1;  
                    array_to_change[1][12] = array_to_change[1][12] + 1;  

                    array_to_change[3][position_intial_colone] = array_to_change[3][position_intial_colone] - 1; 
                    array_to_change[3][position_intial_colone - nombre_dé] = array_to_change[3][position_intial_colone - nombre_dé] + 1; 
                }
                else if(array_to_change[2][position_intial_colone - nombre_dé] == 0){
                    //printf("la3\n"); 
                    array_to_change[2][position_intial_colone - nombre_dé] = 2; 
                    array_to_change[3][position_intial_colone] = array_to_change[3][position_intial_colone] - 1; 
                    array_to_change[3][position_intial_colone - nombre_dé] = 1;  
                }
                else if(array_to_change[2][position_intial_colone - nombre_dé] == 2){
                    //printf("la4\n"); 
                    array_to_change[3][position_intial_colone] = array_to_change[3][position_intial_colone] - 1; 
                    array_to_change[3][position_intial_colone - nombre_dé] = array_to_change[3][position_intial_colone - nombre_dé] + 1; 
                }
                if(array_to_change[3][position_intial_colone] == 0){
                    //printf("la5\n"); 
                    array_to_change[2][position_intial_colone] = 0; 
                }
            }

            else{ 
                //printf("la6\n"); 

                nombre_dé = nombre_dé - position_intial_colone - 1; 
                int position_intial_colone_2 = 0; 

                if((array_to_change[0][position_intial_colone_2 + nombre_dé] == 1)&&(array_to_change[1][position_intial_colone_2 + nombre_dé] == 1)){
                    //printf("la7\n"); 
                    array_to_change[0][position_intial_colone_2 + nombre_dé] = 2;
                    array_to_change[1][position_intial_colone_2 + nombre_dé] = 0;
                    array_to_change[0][12] = 1;  
                    array_to_change[1][12] = array_to_change[1][12] + 1;  

                    array_to_change[3][position_intial_colone] = array_to_change[3][position_intial_colone] - 1; 
                    array_to_change[1][position_intial_colone_2 + nombre_dé] = array_to_change[1][position_intial_colone_2 + nombre_dé] + 1; 
                }
                else if(array_to_change[0][position_intial_colone_2 + nombre_dé] == 0){
                    //printf("la8\n"); 
                    array_to_change[0][position_intial_colone_2 + nombre_dé] = 2; 
                    array_to_change[3][position_intial_colone] = array_to_change[3][position_intial_colone] - 1; 
                    array_to_change[1][position_intial_colone_2 + nombre_dé] = 1;  
                }
                else if(array_to_change[0][position_intial_colone_2 + nombre_dé] == 2){
                    //printf("la9\n"); 
                    array_to_change[3][position_intial_colone] = array_to_change[3][position_intial_colone] - 1; 
                    array_to_change[1][position_intial_colone_2 + nombre_dé] = array_to_change[1][position_intial_colone_2 + nombre_dé] + 1; 
                }
                if(array_to_change[3][position_intial_colone] == 0){
                    //printf("la10\n"); 
                    array_to_change[2][position_intial_colone] = 0; 
                }
                
            }
        }
        else if(position_intial_ligne == 0){
            //printf("la11\n"); 
                if((array_to_change[0][position_intial_colone + nombre_dé] == 1)&&(array_to_change[1][position_intial_colone + nombre_dé] == 1)){
                    //printf("la12\n"); 
                    array_to_change[0][position_intial_colone + nombre_dé] = 2;
                    array_to_change[1][position_intial_colone + nombre_dé] = 0;
                    array_to_change[0][12] = 1;  
                    array_to_change[1][12] = array_to_change[1][12] + 1;  

                    array_to_change[1][position_intial_colone] = array_to_change[1][position_intial_colone] - 1; 
                    array_to_change[1][position_intial_colone + nombre_dé] = array_to_change[1][position_intial_colone + nombre_dé] + 1; 
                }
                else if(array_to_change[0][position_intial_colone + nombre_dé] == 0){
                    //printf("la14\n"); 
                    array_to_change[0][position_intial_colone + nombre_dé] = 2; 
                    array_to_change[1][position_intial_colone] = array_to_change[1][position_intial_colone] - 1; 
                    array_to_change[1][position_intial_colone + nombre_dé] = 1;  
                }
                else if(array_to_change[0][position_intial_colone + nombre_dé] == 2){
                    //printf("la15\n"); 
                    array_to_change[1][position_intial_colone] = array_to_change[1][position_intial_colone] - 1; 
                    array_to_change[1][position_intial_colone + nombre_dé] = array_to_change[1][position_intial_colone + nombre_dé] + 1; 
                }
                if(array_to_change[1][position_intial_colone] == 0){
                    //printf("la16\n"); 
                    array_to_change[0][position_intial_colone] = 0; 
                }
        }

    }

}

int function_is_every_chercker_in_last_box_O(int tavli[4][13]){
    //couleur blanche = 1 d'abord
    int sommeblancs = 0; 
    for(int i = 6; i < 12; i++){
        if(tavli[2][i] == 1){
            sommeblancs = sommeblancs + tavli[3][i]; 
        }
    }
    if(sommeblancs >= 15){
        return 1; 
    }
    else{
        return 0; 
    }

}

int function_is_every_chercker_in_last_box_X(int tavli[4][13]){ 
    //couleur rouge = 2 ensuite
    int sommerouges = 0; 
    for(int i = 6; i < 12; i++){
        if(tavli[0][i] == 2){
            sommerouges = sommerouges + tavli[1][i]; 
        }
    }
    if(sommerouges >= 15){
        return 1; 
    }
    else{
        return 0; 
    }
}

int function_measure_number_of_checker_O(int tavli[4][13]){
    int sommeblancs = 0; 
    for(int i = 0; i < 13; i++){
        if(tavli[0][i] == 1){
            sommeblancs = sommeblancs + tavli[1][i]; 
        }
    }

        for(int i = 0; i < 13; i++){
        if(tavli[2][i] == 1){
            sommeblancs = sommeblancs + tavli[3][i]; 
        }
    }

    return sommeblancs; 
}

int function_measure_number_of_checker_X(int tavli[4][13]){
    int sommerouges = 0; 
    for(int i = 0; i < 13; i++){
        if(tavli[0][i] == 2){
            sommerouges = sommerouges + tavli[1][i]; 
        }
    }

        for(int i = 0; i < 13; i++){
        if(tavli[2][i] == 2){
            sommerouges = sommerouges + tavli[3][i]; 
        }
    }

    return sommerouges; 
}

void number_tavli_generator(int tavli[4][13]){
    printf("\n"); 
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 13; j++){
            printf("%d", tavli[i][j]);  
        }
        printf("\n"); 
    }
    printf("\n"); 
}

int function_is_free(int tavli[4][13], int couleur, int position_intial_ligne, int position_intial_colone, int nombre_dé ){
    if(couleur == 1){
        if(position_intial_ligne == 0){
            if(tavli[0][position_intial_colone] != 1){
                return 0; 
            }
            if(position_intial_colone >= nombre_dé){
                if((tavli[0][position_intial_colone - nombre_dé] == 2) && (tavli[1][position_intial_colone - nombre_dé] > 1)){
                    return 0; 
                }
                else{
                    return 1; 
                }
            }
            else{
                nombre_dé = nombre_dé - position_intial_colone - 1; 
                int position_intial_colone_2 = 0; 
                if((tavli[2][position_intial_colone_2 + nombre_dé] == 2) && (tavli[3][position_intial_colone_2 + nombre_dé] > 1)){
                    return 0; 
                }
                else{
                    return 1; 
                }
            }

        }

        else if(position_intial_ligne == 1){
            if(tavli[2][position_intial_colone] != 1){
                return 0; 
            }
            if(position_intial_colone >= 6 ){
                if(nombre_dé > ( 11 - position_intial_colone) ){
                    return 0; 
                }
                else{
                    if((tavli[2][position_intial_colone + nombre_dé] == 2)&&(tavli[3][position_intial_colone + nombre_dé] > 1)){
                        return 0;  
                    }   
                    else{
                        return 1; 
                    }                 
                }

            }
            else if(position_intial_colone < 6){
                if((tavli[2][position_intial_colone + nombre_dé] == 2)&&(tavli[3][position_intial_colone + nombre_dé] > 1)){
                    return 0;  
                }
                else{
                    return 1; 
                }
            }
                
        }

    }

    else if (couleur == 2){
        if(position_intial_ligne == 1){
            if(tavli[2][position_intial_colone] != 2){
                return 0; 
            }

            if(position_intial_colone >= nombre_dé){

                if((tavli[2][position_intial_colone - nombre_dé] == 1)&&(tavli[3][position_intial_colone - nombre_dé] > 1)){
                    return 0; 
                }
                else{
                    return 1; 
                }
            }

            else{ 
                nombre_dé = nombre_dé - position_intial_colone - 1; 
                int position_intial_colone_2 = 0; 

                if((tavli[0][position_intial_colone_2 + nombre_dé] == 1)&&(tavli[1][position_intial_colone_2 + nombre_dé] > 1)){
                    return 0; 
                }
                return 1;    
            }
        }

        else if(position_intial_ligne == 0){
            if(tavli[0][position_intial_colone] != 2){
                return 0; 
            }
            if(position_intial_colone >= 6 ){
                if(nombre_dé > ( 11 - position_intial_colone) ){
                    return 0; 
                }
                else{
                    if((tavli[0][position_intial_colone + nombre_dé] == 1)&&(tavli[1][position_intial_colone + nombre_dé] > 1)){
                        return 0;  
                    }   
                    else{
                        return 1; 
                    }                 
                }
            }
            else if(position_intial_colone < 6){
                if((tavli[0][position_intial_colone + nombre_dé] == 1)&&(tavli[1][position_intial_colone + nombre_dé] > 1)){
                    return 0;  
                }
                else{
                    return 1; 
                }
            }


        }

    }  
}

int function_checker_out(int tavli[4][13], int couleur){
    if(couleur == 1){
        if(tavli[0][12] == 1){
            return 1; 
        }
        else{
            return 0; 
        }
    }
    else if(couleur == 2){
        if(tavli[2][12] == 2){
            return 1; 
        }
        else{
            return 0; 
        }
    }
}

int function_find__number_position_to_play(int tavli[3][13], int couleur, int dice){
    int mesure_nombre_to_play = 0; 
    int position_intial_ligne; 
    int position_intial_colone; 

    if(couleur == 1){
        for(int i = 0; i < 12; i++){
            position_intial_ligne = 0; 
            position_intial_colone = i; 
            if( (tavli[0][i] == couleur) &&(function_is_free(tavli, couleur, position_intial_ligne, position_intial_colone, dice )) ){
                mesure_nombre_to_play = mesure_nombre_to_play + 1; 
            }
        }
        for(int i = 0; i < 12; i++){
            position_intial_ligne = 1; 
            position_intial_colone = i;         
            if( (tavli[2][i] == couleur) &&(function_is_free(tavli, couleur, position_intial_ligne, position_intial_colone, dice )) ){
                mesure_nombre_to_play = mesure_nombre_to_play + 1; 
            }
        }        
    }
    else if(couleur == 2){
        for(int i = 0; i < 12; i++){
            position_intial_ligne = 0; 
            position_intial_colone = i; 
            if( (tavli[0][i] == couleur) &&(function_is_free(tavli, couleur, position_intial_ligne, position_intial_colone, dice )) ){
                mesure_nombre_to_play = mesure_nombre_to_play + 1; 
            }
        }
        for(int i = 0; i < 12; i++){
            position_intial_ligne = 1; 
            position_intial_colone = i;         
            if( (tavli[2][i] == couleur) &&(function_is_free(tavli, couleur, position_intial_ligne, position_intial_colone, dice )) ){
                mesure_nombre_to_play = mesure_nombre_to_play + 1; 
            }
        }        
        
    }

    return mesure_nombre_to_play; 
}

int function_return_dice_non_zero_from_array(int array_dice[4]){

    int retour_de_fonction; 
    for(int i = 0; i < 4; i++){
        if(array_dice[i] != 0){
            retour_de_fonction = array_dice[i];
            array_dice[i] = 0; 
            return retour_de_fonction; 
        }
    }
}

void fonction_retourne_position_possible_pour_changement(int tavli[4][13], int couleur, int nombre_dé, int taille_tableau, int **position_possibles ){
    //printf("taille du tableau de position disponibles = %d\n", taille_tableau); 
    int position_intial_ligne; 
    int position_intial_colone; 
    int position_dans_tableau_taille_possible = 0; 

    if(couleur == 1){
        for(int i = 0; i < 12; i++){
            position_intial_ligne = 0; 
            position_intial_colone = i; 
            if( (tavli[0][i] == couleur) && (function_is_free(tavli, couleur, position_intial_ligne, position_intial_colone, nombre_dé ) == 1)){
                position_possibles[0][position_dans_tableau_taille_possible] = position_intial_ligne; 
                position_possibles[1][position_dans_tableau_taille_possible] = position_intial_colone; 
                position_dans_tableau_taille_possible = position_dans_tableau_taille_possible + 1; 
            }
        }
        for(int i = 0; i < 12; i++){
            position_intial_ligne = 1; 
            position_intial_colone = i; 
            if( (tavli[2][i] == couleur) && ((function_is_free(tavli, couleur, position_intial_ligne, position_intial_colone, nombre_dé ) == 1)) ){
                position_possibles[0][position_dans_tableau_taille_possible] = position_intial_ligne; 
                position_possibles[1][position_dans_tableau_taille_possible] = position_intial_colone; 
                position_dans_tableau_taille_possible = position_dans_tableau_taille_possible + 1;   
            }
        }
    }

    else if(couleur == 2){
        for(int i = 0; i < 12; i++){
            position_intial_ligne = 0; 
            position_intial_colone = i; 
            if( (tavli[0][i] == couleur) && (function_is_free(tavli, couleur, position_intial_ligne, position_intial_colone, nombre_dé ) == 1)){
                position_possibles[0][position_dans_tableau_taille_possible] = position_intial_ligne; 
                position_possibles[1][position_dans_tableau_taille_possible] = position_intial_colone; 
                position_dans_tableau_taille_possible = position_dans_tableau_taille_possible + 1; 
            }
        }
        for(int i = 0; i < 12; i++){
            position_intial_ligne = 1; 
            position_intial_colone = i; 
            if( (tavli[2][i] == couleur) && ((function_is_free(tavli, couleur, position_intial_ligne, position_intial_colone, nombre_dé ) == 1)) ){
                position_possibles[0][position_dans_tableau_taille_possible] = position_intial_ligne; 
                position_possibles[1][position_dans_tableau_taille_possible] = position_intial_colone; 
                position_dans_tableau_taille_possible = position_dans_tableau_taille_possible + 1;   
            }
        }            

    }


   



    for(int i = 0; i < taille_tableau; i++){
        //printf("%d ", position_possibles[0][i] ); 
    }
    //printf("\n"); 
    for(int i = 0; i < taille_tableau; i++){
        //printf("%d ", position_possibles[1][i] );         
    }
    //printf("\n"); 
    //printf("\n"); 

}

int bot_simulation(int tavli[4][13], int couleur){
    srand(time(0)); // Set a new seed for the random number generator
    int lower = 1; 
    int upper = 6; 
    int tavli2[4][13]; 
    int dice1; 
    int dice2;    
    int coup_joués = 0; 
    int dé_possibles = 2; 
    int dice_array[4]; 
    int nombre_to_play; 
    int num_values; 
    int random_index; 
    int random_value; 
    int position_intial_ligne; 
    int position_intial_colone; 
    int taille_tableau; 
    int size; 
    int height = 2; 
    int dice1_or_dice2; 

    int tour_joue_O = 0; 
    int tour_joue_X = 0; 


    for(int i = 0;i < 4; i++){
        for(int j = 0; j < 13; j++){
            tavli2[i][j] = tavli[i][j]; 
        }
    }


    while(( function_is_every_chercker_in_last_box_O(tavli2) != 1) && (function_is_every_chercker_in_last_box_X(tavli2) != 1)){
        lower = 1; 
        upper = 6; 

        //Tour de O; 
        if(couleur == 1){
            tour_joue_O = tour_joue_O + 1; 
            dé_possibles = 2; 
            //printf("couleur = O \n"); 
            dice1 = generator_dice(lower, upper); 
            dice2 = generator_dice(lower, upper); 
            //printf("dés = %d et %d\n", dice1, dice2); 
            dice_array[3] = dice1; 
            dice_array[2] = dice2; 
            dice_array[1] = dice1; 
            dice_array[0] = dice2; 

            if(dice1 == dice2){
                dé_possibles = 4; 
                dice_array[3] = dice1; 
                dice_array[2] = dice2; 
            }
            

            int position_initiale_ligne_blanc = 0; 
            int position_initiale_colone_blanc = 12; 
            
            for(int i = dé_possibles; i > 0; i--){



                //printf("nombre de checkers blancs = %d\n", function_measure_number_of_checker_O( tavli2 )); 
                //printf("nombre de checkers rouges = %d\n", function_measure_number_of_checker_X( tavli2 )); 
                
                if( function_checker_out(tavli2, couleur) == 1){
                    //printf("checkerdehors\n"); 
                    if(dice1 == dice2){
                        //printf("dés égaux\n"); 
                        if( function_is_free(tavli2, couleur,position_initiale_ligne_blanc, position_initiale_colone_blanc, dice_array[i-1] ) == 1){
                            position_changer(tavli2, couleur, position_initiale_ligne_blanc,position_initiale_colone_blanc, dice_array[i-1] ); 
                            dé_possibles = dé_possibles - 1; 
                            //picture_generator(tavli2);
                        }
                        else{
                            i = 0; 
                            dé_possibles = 0; 
                            //picture_generator(tavli2); 
                        }
                    }
                    else{  //dice1 different than dice2 ; 
                        //printf("dés différents\n"); 
                        if( function_is_free(tavli2, couleur,position_initiale_ligne_blanc, position_initiale_colone_blanc, dice_array[i+1] ) == 1){
                            //printf("cas1\n"); 
                            position_changer(tavli2, couleur, position_initiale_ligne_blanc,position_initiale_colone_blanc, dice_array[i+1] ); 
                            dé_possibles = dé_possibles - 1; 
                            //picture_generator(tavli2);
                            dice_array[1] = dice2; 
                        }
                        else if (function_is_free(tavli2, couleur,position_initiale_ligne_blanc, position_initiale_colone_blanc, dice_array[i] ) == 1){
                            //printf("cas2\n"); 
                            position_changer(tavli2, couleur, position_initiale_ligne_blanc,position_initiale_colone_blanc, dice_array[i] ); 
                            //picture_generator(tavli2);
                            dé_possibles = dé_possibles - 1 ; 
                            dice_array[0] = dice1;   
                            dice_array[2] =  dice1; 
                        }
                        else{
                            //printf("cas3\n"); 
                            i = 0; 
                            dé_possibles = 0; 
                            //picture_generator(tavli2); 
                        } 

                    }
                }


                else{
                    //printf("pas de checkers out\n"); 
                    if(dice1 == dice2){
                        //printf("dés égaux\n"); 
                        random_value = dice1; 
                        taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice1);

                        if(taille_tableau == 0){
                            i = 0; 
                        }
                        else{
                            size = taille_tableau;
                            int **position_possibles_dice_egaux = malloc(sizeof(int *) * height); 
                            for(int i = 0; i < height; i++){
                                position_possibles_dice_egaux[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                            }
                            for(int i = 0; i < 2; i++){
                                for(int j = 0; j < taille_tableau; j++){
                                    position_possibles_dice_egaux[i][j] = 0; 
                                }
                            }
                            fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice1, taille_tableau, position_possibles_dice_egaux);
                            lower = 0; 
                            upper = taille_tableau - 1; 
                            random_index = generator_dice(lower, upper); 
                            position_intial_ligne = position_possibles_dice_egaux[0][random_index]; 
                            position_intial_colone = position_possibles_dice_egaux[1][random_index]; 
                            position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone , dice1);

                            for (int i = 0; i < height; i++) {
                                free(position_possibles_dice_egaux[i]);
                            }
                            // Free the array of pointers
                            free(position_possibles_dice_egaux);  
                            dé_possibles = dé_possibles - 1; 
                            //picture_generator(tavli2);
                        }
                    }

                    else{
                        //printf("dés différents\n"); 
                        if(dé_possibles == 2){
                            //printf("il nous reste 2 chances de jouer\n"); 

                            if( function_find__number_position_to_play(tavli2, couleur, dice1) > 0 && function_find__number_position_to_play(tavli2, couleur, dice2) > 0 ){
                                //printf("on peut jouer les deux dés\n"); 
                                lower = 1; 
                                upper = 2; 
                                dice1_or_dice2 =  generator_dice(lower, upper); 

                                if( dice1_or_dice2 == 1 ){
                                    //printf("on joue le dé 1\n"); 
                                    dice_array[0] = dice2; 
                                    taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice1);                           
                                    size = taille_tableau;
                                    int **position_possibles_dice_cas_1a = malloc(sizeof(int *) * height); 
                                    for(int i = 0; i < height; i++){
                                        position_possibles_dice_cas_1a[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                                    }
                                    for(int i = 0; i < 2; i++){
                                        for(int j = 0; j < taille_tableau; j++){
                                            position_possibles_dice_cas_1a[i][j] = 0; 
                                        }
                                    }
                                    fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice1, taille_tableau, position_possibles_dice_cas_1a);
                                    lower = 0; 
                                    upper = taille_tableau - 1; 
                                    random_index = generator_dice(lower, upper); 
                                    position_intial_ligne = position_possibles_dice_cas_1a[0][random_index]; 
                                    position_intial_colone = position_possibles_dice_cas_1a[1][random_index]; 
                                    position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone, dice1);

                                    for (int i = 0; i < height; i++) {
                                        free(position_possibles_dice_cas_1a[i]);
                                    }
                                    // Free the array of pointers
                                    free(position_possibles_dice_cas_1a);     
                                    //picture_generator(tavli2);
                                }

                                else if( dice1_or_dice2 == 2 ){
                                    //printf("on joue le dé 2\n"); 
                                    dice_array[0] = dice1; 

                                    taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice2);  
                                    size = taille_tableau;
                                    int **position_possibles_dice_cas_1b = malloc(sizeof(int *) * height); 
                                    for(int i = 0; i < height; i++){
                                        position_possibles_dice_cas_1b[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                                    }
                                    for(int i = 0; i < 2; i++){
                                        for(int j = 0; j < taille_tableau; j++){
                                            position_possibles_dice_cas_1b[i][j] = 0; 
                                        }
                                    }


                                    fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice2, taille_tableau, position_possibles_dice_cas_1b);

                                    lower = 0; 
                                    upper = taille_tableau - 1; 
                                    random_index = generator_dice(lower, upper); 
                                    position_intial_ligne = position_possibles_dice_cas_1b[0][random_index]; 
                                    position_intial_colone = position_possibles_dice_cas_1b[1][random_index]; 
                                    position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone, dice2);

                                    for (int i = 0; i < height; i++) {
                                        free(position_possibles_dice_cas_1b[i]);
                                    }
                                    // Free the array of pointers
                                    free(position_possibles_dice_cas_1b); 
                                    //picture_generator(tavli2);                                   
                                }
                                dé_possibles = dé_possibles - 1;                                
                            }


                            else if( ( function_find__number_position_to_play(tavli2, couleur, dice1) > 0 )&&( function_find__number_position_to_play(tavli2, couleur, dice2) == 0 ) ){
                                //printf("on ne peut jouer que le dés 1\n"); 
                                taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice1);                           
                                size = taille_tableau;
                                int **position_possibles_dice_cas_2 = malloc(sizeof(int *) * height); 
                                for(int i = 0; i < height; i++){
                                    position_possibles_dice_cas_2[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                                }
                                for(int i = 0; i < 2; i++){
                                    for(int j = 0; j < taille_tableau; j++){
                                        position_possibles_dice_cas_2[i][j] = 0; 
                                    }
                                }
                                fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice1, taille_tableau, position_possibles_dice_cas_2);
                                lower = 0; 
                                upper = taille_tableau - 1; 
                                random_index = generator_dice(lower, upper); 
                                position_intial_ligne = position_possibles_dice_cas_2[0][random_index]; 
                                position_intial_colone = position_possibles_dice_cas_2[1][random_index]; 
                                position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone , dice1);

                                for (int i = 0; i < height; i++) {
                                    free(position_possibles_dice_cas_2[i]);
                                }
                                // Free the array of pointers
                                free(position_possibles_dice_cas_2);
                                //picture_generator(tavli2);
                                dice_array[0] = dice2; 
                                dé_possibles = dé_possibles - 1; 
                        
        

                            }

                            else if( ( function_find__number_position_to_play(tavli2, couleur, dice1) == 0 )&&( function_find__number_position_to_play(tavli2, couleur, dice2) > 0) ){
                                //printf("on ne peut jouer que le dé 2\n"); 
                                taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice2);
                                size = taille_tableau;
                                int **position_possibles_dice_cas_3 = malloc(sizeof(int *) * height); 
                                for(int i = 0; i < height; i++){
                                    position_possibles_dice_cas_3[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                                }
                                for(int i = 0; i < 2; i++){
                                    for(int j = 0; j < taille_tableau; j++){
                                        position_possibles_dice_cas_3[i][j] = 0; 
                                    }
                                }
                                fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice2, taille_tableau, position_possibles_dice_cas_3);
                                lower = 0; 
                                upper = taille_tableau - 1; 
                                random_index = generator_dice(lower, upper); 
                                position_intial_ligne = position_possibles_dice_cas_3[0][random_index]; 
                                position_intial_colone = position_possibles_dice_cas_3[1][random_index]; 
                                position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone , dice2);

                                for (int i = 0; i < height; i++) {
                                    free(position_possibles_dice_cas_3[i]);
                                }
                                // Free the array of pointers
                                free(position_possibles_dice_cas_3);
                                //picture_generator(tavli2);
                                dice_array[0] = dice2;
                                dé_possibles = dé_possibles - 1; 
                            }
                            else if( ( function_find__number_position_to_play(tavli2, couleur, dice1) == 0 )&&( function_find__number_position_to_play(tavli2, couleur, dice2) == 0 ) ){
                                //picture_generator(tavli2);
                                i = 0; 
                            }



                        }

                        else if(dé_possibles == 1){
                            //printf("on ne peut jouer que une chance\n"); 

                            random_value = dice_array[0]; 
                            taille_tableau = function_find__number_position_to_play(tavli2, couleur, random_value);

                            if(taille_tableau == 0){
                                i = 0; 
                            }
                            else{
                                size = taille_tableau;
                                int **position_possibles_dice_diff = malloc(sizeof(int *) * height); 
                                for(int i = 0; i < height; i++){
                                    position_possibles_dice_diff[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                                }
                                for(int i = 0; i < 2; i++){
                                    for(int j = 0; j < taille_tableau; j++){
                                        position_possibles_dice_diff[i][j] = 0; 
                                    }
                                }
                                fonction_retourne_position_possible_pour_changement(tavli2, couleur, random_value, taille_tableau, position_possibles_dice_diff);
                                lower = 0; 
                                upper = taille_tableau - 1; 
                                random_index = generator_dice(lower, upper); 
                                position_intial_ligne = position_possibles_dice_diff[0][random_index]; 
                                position_intial_colone = position_possibles_dice_diff[1][random_index]; 
                                position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone , random_value);

                                for (int i = 0; i < height; i++) {
                                    free(position_possibles_dice_diff[i]);
                                }
                                // Free the array of pointers
                                free(position_possibles_dice_diff); 
                                //picture_generator(tavli2); 
                                i = 0; 
                            }

                        }
                    }
                }
                
                
                if(( function_is_every_chercker_in_last_box_O(tavli2) == 1) || (function_is_every_chercker_in_last_box_X(tavli2) == 1)){
                    break; 
                }
            }


        
            couleur = 2; 
        }
    
        //Tour de X; 
        else if(couleur == 2){
            tour_joue_X = tour_joue_X + 1; 
            dé_possibles = 2; 
            //printf("couleur = X \n"); 
            dice1 = generator_dice(lower, upper); 
            dice2 = generator_dice(lower, upper); 
            //printf("dés = %d et %d\n", dice1, dice2); 
            dice_array[3] = dice1; 
            dice_array[2] = dice2; 
            dice_array[1] = dice1; 
            dice_array[0] = dice2; 

            if(dice1 == dice2){
                dé_possibles = 4; 
                dice_array[3] = dice1; 
                dice_array[2] = dice2; 
            }
            

            int position_initiale_ligne_blanc = 1; 
            int position_initiale_colone_blanc = 12; 
            
            for(int i = dé_possibles; i > 0; i--){
              
                //printf("nombre de checkers blancs = %d\n", function_measure_number_of_checker_O( tavli2 )); 
                //printf("nombre de checkers rouges = %d\n", function_measure_number_of_checker_X( tavli2 ));                 
                if( function_checker_out(tavli2, couleur) == 1){
                    //printf("checkerdehors\n"); 
                    if(dice1 == dice2){
                        //printf("dés égaux\n"); 
                        if( function_is_free(tavli2, couleur,position_initiale_ligne_blanc, position_initiale_colone_blanc, dice_array[i-1] ) == 1){
                            position_changer(tavli2, couleur, position_initiale_ligne_blanc,position_initiale_colone_blanc, dice_array[i-1] ); 
                            dé_possibles = dé_possibles - 1; 
                            //picture_generator(tavli2);
                        }
                        else{
                            i = 0; 
                            dé_possibles = 0; 
                            //picture_generator(tavli2); 
                        }
                    }
                    else{  //dice1 different than dice2 ; 
                        //printf("dés différents\n"); 
                        if( function_is_free(tavli2, couleur,position_initiale_ligne_blanc, position_initiale_colone_blanc, dice_array[i+1] ) == 1){
                            //printf("cas1\n"); 
                            position_changer(tavli2, couleur, position_initiale_ligne_blanc,position_initiale_colone_blanc, dice_array[i+1] ); 
                            dé_possibles = dé_possibles - 1; 
                            //picture_generator(tavli2);
                            dice_array[1] = dice2;
                        }
                        else if (function_is_free(tavli2, couleur,position_initiale_ligne_blanc, position_initiale_colone_blanc, dice_array[i] ) == 1){
                            //printf("cas2\n"); 
                            position_changer(tavli2, couleur, position_initiale_ligne_blanc,position_initiale_colone_blanc, dice_array[i] ); 
                            //picture_generator(tavli2);
                            dé_possibles = dé_possibles - 1 ; 
                            dice_array[0] = dice1;  
                            dice_array[2] = dice1;   
                        }
                        else{
                            //printf("cas3\n"); 
                            i = 0; 
                            dé_possibles = 0; 
                            //picture_generator(tavli2); 
                        } 
                    }
                }


                else{
                    //printf("pas de checkers out\n"); 
                    if(dice1 == dice2){
                        //printf("dés égaux\n"); 
                        random_value = dice1; 
                        taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice1);

                        if(taille_tableau == 0){
                            i = 0; 
                        }
                        else{
                            size = taille_tableau;
                            int **position_possibles_dice_egaux = malloc(sizeof(int *) * height); 
                            for(int i = 0; i < height; i++){
                                position_possibles_dice_egaux[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                            }
                            for(int i = 0; i < 2; i++){
                                for(int j = 0; j < taille_tableau; j++){
                                    position_possibles_dice_egaux[i][j] = 0; 
                                }
                            }
                            fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice1, taille_tableau, position_possibles_dice_egaux);
                            lower = 0; 
                            upper = taille_tableau - 1; 
                            random_index = generator_dice(lower, upper); 
                            position_intial_ligne = position_possibles_dice_egaux[0][random_index]; 
                            position_intial_colone = position_possibles_dice_egaux[1][random_index]; 
                            position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone , dice1);

                            for (int i = 0; i < height; i++) {
                                free(position_possibles_dice_egaux[i]);
                            }
                            // Free the array of pointers
                            free(position_possibles_dice_egaux);  
                            dé_possibles = dé_possibles - 1; 
                            //picture_generator(tavli2);
                        }
                    }

                    else{
                        //printf("dés différents\n"); 
                        if(dé_possibles == 2){
                            //printf("il nous reste 2 chances de jouer\n"); 

                            if( function_find__number_position_to_play(tavli2, couleur, dice1) > 0 && function_find__number_position_to_play(tavli2, couleur, dice2) > 0 ){
                                //printf("on peut jouer les deux dés\n"); 
                                lower = 1; 
                                upper = 2; 
                                dice1_or_dice2 =  generator_dice(lower, upper); 

                                if( dice1_or_dice2 == 1 ){
                                    //printf("on joue le dé 1\n"); 
                                    dice_array[0] = dice2; 
                                    taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice1);                           
                                    size = taille_tableau;
                                    int **position_possibles_dice_cas_1a = malloc(sizeof(int *) * height); 
                                    for(int i = 0; i < height; i++){
                                        position_possibles_dice_cas_1a[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                                    }
                                    for(int i = 0; i < 2; i++){
                                        for(int j = 0; j < taille_tableau; j++){
                                            position_possibles_dice_cas_1a[i][j] = 0; 
                                        }
                                    }
                                    fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice1, taille_tableau, position_possibles_dice_cas_1a);
                                    lower = 0; 
                                    upper = taille_tableau - 1; 
                                    random_index = generator_dice(lower, upper); 
                                    position_intial_ligne = position_possibles_dice_cas_1a[0][random_index]; 
                                    position_intial_colone = position_possibles_dice_cas_1a[1][random_index]; 
                                    position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone, dice1);

                                    for (int i = 0; i < height; i++) {
                                        free(position_possibles_dice_cas_1a[i]);
                                    }
                                    // Free the array of pointers
                                    free(position_possibles_dice_cas_1a);     
                                    //picture_generator(tavli2);
                                }

                                else if( dice1_or_dice2 == 2 ){
                                    //printf("on joue le dé 2\n"); 
                                    dice_array[0] = dice1; 

                                    taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice2);  
                                    size = taille_tableau;
                                    int **position_possibles_dice_cas_1b = malloc(sizeof(int *) * height); 
                                    for(int i = 0; i < height; i++){
                                        position_possibles_dice_cas_1b[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                                    }
                                    for(int i = 0; i < 2; i++){
                                        for(int j = 0; j < taille_tableau; j++){
                                            position_possibles_dice_cas_1b[i][j] = 0; 
                                        }
                                    }


                                    fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice2, taille_tableau, position_possibles_dice_cas_1b);

                                    lower = 0; 
                                    upper = taille_tableau - 1; 
                                    random_index = generator_dice(lower, upper); 
                                    position_intial_ligne = position_possibles_dice_cas_1b[0][random_index]; 
                                    position_intial_colone = position_possibles_dice_cas_1b[1][random_index]; 
                                    position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone, dice2);

                                    for (int i = 0; i < height; i++) {
                                        free(position_possibles_dice_cas_1b[i]);
                                    }
                                    // Free the array of pointers
                                    free(position_possibles_dice_cas_1b); 
                                    //picture_generator(tavli2);                                   
                                }
                                dé_possibles = dé_possibles - 1;                                
                            }


                            else if( ( function_find__number_position_to_play(tavli2, couleur, dice1) > 0 )&&( function_find__number_position_to_play(tavli2, couleur, dice2) == 0 ) ){
                                //printf("on ne peut jouer que le dés 1\n"); 
                                taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice1);                           
                                size = taille_tableau;
                                int **position_possibles_dice_cas_2 = malloc(sizeof(int *) * height); 
                                for(int i = 0; i < height; i++){
                                    position_possibles_dice_cas_2[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                                }
                                for(int i = 0; i < 2; i++){
                                    for(int j = 0; j < taille_tableau; j++){
                                        position_possibles_dice_cas_2[i][j] = 0; 
                                    }
                                }
                                fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice1, taille_tableau, position_possibles_dice_cas_2);
                                lower = 0; 
                                upper = taille_tableau - 1; 
                                random_index = generator_dice(lower, upper); 
                                position_intial_ligne = position_possibles_dice_cas_2[0][random_index]; 
                                position_intial_colone = position_possibles_dice_cas_2[1][random_index]; 
                                position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone , dice1);

                                for (int i = 0; i < height; i++) {
                                    free(position_possibles_dice_cas_2[i]);
                                }
                                // Free the array of pointers
                                free(position_possibles_dice_cas_2);
                                //picture_generator(tavli2);
                                dice_array[0] = dice2; 
                                dé_possibles = dé_possibles - 1; 
                        
        

                            }

                            else if( ( function_find__number_position_to_play(tavli2, couleur, dice1) == 0 )&&( function_find__number_position_to_play(tavli2, couleur, dice2) > 0) ){
                                //printf("on ne peut jouer que le dé 2\n"); 
                                taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice2);
                                size = taille_tableau;
                                int **position_possibles_dice_cas_3 = malloc(sizeof(int *) * height); 
                                for(int i = 0; i < height; i++){
                                    position_possibles_dice_cas_3[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                                }
                                for(int i = 0; i < 2; i++){
                                    for(int j = 0; j < taille_tableau; j++){
                                        position_possibles_dice_cas_3[i][j] = 0; 
                                    }
                                }
                                fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice2, taille_tableau, position_possibles_dice_cas_3);
                                lower = 0; 
                                upper = taille_tableau - 1; 
                                random_index = generator_dice(lower, upper); 
                                position_intial_ligne = position_possibles_dice_cas_3[0][random_index]; 
                                position_intial_colone = position_possibles_dice_cas_3[1][random_index]; 
                                position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone , dice2);

                                for (int i = 0; i < height; i++) {
                                    free(position_possibles_dice_cas_3[i]);
                                }
                                // Free the array of pointers
                                free(position_possibles_dice_cas_3);
                                //picture_generator(tavli2);
                                dice_array[0] = dice2;
                                dé_possibles = dé_possibles - 1; 
                            }
                            else if( ( function_find__number_position_to_play(tavli2, couleur, dice1) == 0 )&&( function_find__number_position_to_play(tavli2, couleur, dice2) == 0 ) ){
                                //picture_generator(tavli2);
                                i = 0; 
                            }



                        }

                        else if(dé_possibles == 1){
                            //printf("on ne peut jouer que une chance\n"); 

                            random_value = dice_array[0]; 
                            taille_tableau = function_find__number_position_to_play(tavli2, couleur, random_value);

                            if(taille_tableau == 0){
                                i = 0; 
                            }
                            else{
                                size = taille_tableau;
                                int **position_possibles_dice_diff = malloc(sizeof(int *) * height); 
                                for(int i = 0; i < height; i++){
                                    position_possibles_dice_diff[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                                }
                                for(int i = 0; i < 2; i++){
                                    for(int j = 0; j < taille_tableau; j++){
                                        position_possibles_dice_diff[i][j] = 0; 
                                    }
                                }
                                fonction_retourne_position_possible_pour_changement(tavli2, couleur, random_value, taille_tableau, position_possibles_dice_diff);
                                lower = 0; 
                                upper = taille_tableau - 1; 
                                random_index = generator_dice(lower, upper); 
                                position_intial_ligne = position_possibles_dice_diff[0][random_index]; 
                                position_intial_colone = position_possibles_dice_diff[1][random_index]; 
                                position_changer(tavli2, couleur, position_intial_ligne,position_intial_colone , random_value);

                                for (int i = 0; i < height; i++) {
                                    free(position_possibles_dice_diff[i]);
                                }
                                // Free the array of pointers
                                free(position_possibles_dice_diff); 
                                //picture_generator(tavli2); 
                                i = 0; 
                            }

                        }
                    }
                }
                
                

            }

                if(( function_is_every_chercker_in_last_box_O(tavli2) == 1) || (function_is_every_chercker_in_last_box_X(tavli2) == 1)){
                    break; 
                }
        
            couleur = 1; 
        }


    }

    //picture_generator(tavli2); 

    if( (function_is_every_chercker_in_last_box_O(tavli2) == 1) && (function_is_every_chercker_in_last_box_X(tavli2) != 1) ){
        return 1; 
    }
    else if( (function_is_every_chercker_in_last_box_X(tavli2) == 1) && (function_is_every_chercker_in_last_box_O(tavli2) != 1) ){
        return 2; 
    }
    else {
        return 3; 
    }

}

int fonction_simulation_multiple(int tavli[4][13], int couleur, int dice1, int dice2){

    int nombre_essai = 75000; 
    int tavli2[4][13]; 
    int tavli3[4][13];
    int tavli4[4][13]; 

    int tavli5[4][13];
    int tavli6[4][13];
    int tavli7[4][13];
    int tavli8[4][13];

    int position_intial_ligne; 
    int position_intial_colone; 
    int position_intial_ligne_2; 
    int position_intial_colone_2;
    int height = 2; 
    int taille_tableau_2; 
    int partie_gagnee = 0; 
    int valeur_simulation;
    float partie_gagnee_pourcentage;  
    int taille_tableau; 

    int position_ligne_checker_out; 
    int position_colone_checker_out; 

    for(int i = 0;i < 4; i++){
        for(int j = 0; j < 13; j++){
            tavli2[i][j] = tavli[i][j]; 
        }
    }

    if( function_checker_out(tavli2, couleur) == 1 ){   
        printf("cas spécifique car il y a un checker dehors\n"); 

        //il y a des checkers dehors; 
        if(couleur == 1){
            position_ligne_checker_out = 0; 
            position_colone_checker_out = 12; 
            if(tavli2[1][12] > 1){
                if( (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) == 1) && (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) == 1) ){
                    printf("Les deux dés peuvent être joués, peut importe l'ordre\n"); 
                }
                else if( (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) == 1) && (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) != 1) ){
                    printf("seul le premier dé est une possibilité\n"); 
                }
                else if( (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) != 1) && (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) == 1) ){
                    printf("seul le deuxième dé est une possibilité\n"); 
                }
            }
            else{
                if( ( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) == 1 )&&( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) == 1 ) ){
                    printf("deux dés possibles pour sortir le checker\n"); 



                //on sort d'abord avec le checker 1 ; 
                for(int i = 0;i < 4; i++){
                    for(int j = 0; j < 13; j++){
                        tavli5[i][j] = tavli[i][j]; 
                    }
                }                    
                position_changer(tavli5, couleur, position_ligne_checker_out,position_colone_checker_out , dice1);

                taille_tableau = function_find__number_position_to_play(tavli5, couleur, dice1);                           
                int **position_possibles_checker_1 = malloc(sizeof(int *) * height); 
                for(int i = 0; i < height; i++){
                    position_possibles_checker_1[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                }

                for(int i = 0; i < 2; i++){
                    for(int j = 0; j < taille_tableau; j++){
                        position_possibles_checker_1[i][j] = 0; 
                    }
                }
                fonction_retourne_position_possible_pour_changement(tavli5, couleur, dice2, taille_tableau, position_possibles_checker_1);

                for(int m = 0 ; m < taille_tableau; m++){

                    for(int i = 0;i < 4; i++){
                        for(int j = 0; j < 13; j++){
                        tavli6[i][j] = tavli5[i][j]; 
                        }
                    }

                    //printf("tour numéro %d \n", m); 
                    position_intial_ligne = position_possibles_checker_1[0][m]; 
                    position_intial_colone = position_possibles_checker_1[1][m];
                    position_changer(tavli6, couleur, position_intial_ligne,position_intial_colone , dice2); 

                    for(int h = 0; h < nombre_essai; h++){
                        valeur_simulation = bot_simulation(tavli6, couleur); 
                        //printf("valeur simulation = %d et couleur = %d\n", valeur_simulation, couleur); 

                        if( valeur_simulation == 1){
                            partie_gagnee = partie_gagnee + 1; 
                        }
                    }     
                    partie_gagnee_pourcentage = (float)partie_gagnee/nombre_essai; 
                    printf("dé 1 (%d) :  ligne %d colone %d, dé 2 (%d) : ligne %d colone %d, score : %d, pourcentage = %f \n", dice1, position_ligne_checker_out, position_colone_checker_out, dice2,  position_intial_ligne, position_intial_colone, partie_gagnee, partie_gagnee_pourcentage);
                    partie_gagnee = 0 ;    
                }                 



                    // on sort d'abord avec le checker 2 ; 
                for(int i = 0;i < 4; i++){
                    for(int j = 0; j < 13; j++){
                        tavli7[i][j] = tavli[i][j]; 
                    }
                }                    
                position_changer(tavli7, couleur, position_ligne_checker_out,position_colone_checker_out , dice2);

                taille_tableau = function_find__number_position_to_play(tavli7, couleur, dice2);                           
                int **position_possibles_checker_2 = malloc(sizeof(int *) * height); 
                for(int i = 0; i < height; i++){
                    position_possibles_checker_2[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                }

                for(int i = 0; i < 2; i++){
                    for(int j = 0; j < taille_tableau; j++){
                        position_possibles_checker_2[i][j] = 0; 
                    }
                }
                fonction_retourne_position_possible_pour_changement(tavli5, couleur, dice1, taille_tableau, position_possibles_checker_2);

                for(int m = 0 ; m < taille_tableau; m++){

                    for(int i = 0;i < 4; i++){
                        for(int j = 0; j < 13; j++){
                        tavli8[i][j] = tavli7[i][j]; 
                        }
                    }

                    //printf("tour numéro %d \n", m); 
                    position_intial_ligne = position_possibles_checker_2[0][m]; 
                    position_intial_colone = position_possibles_checker_2[1][m];
                    position_changer(tavli8, couleur, position_intial_ligne,position_intial_colone , dice1); 

                    for(int h = 0; h < nombre_essai; h++){
                        valeur_simulation = bot_simulation(tavli8, couleur); 
                        //printf("valeur simulation = %d et couleur = %d\n", valeur_simulation, couleur); 

                        if( valeur_simulation == 1){
                            partie_gagnee = partie_gagnee + 1; 
                        }
                    }     
                    partie_gagnee_pourcentage = (float)partie_gagnee/nombre_essai; 
                    printf("dé 1 (%d) :  ligne %d colone %d, dé 2 (%d) : ligne %d colone %d, score : %d, pourcentage = %f \n", dice2, position_ligne_checker_out, position_colone_checker_out, dice1,  position_intial_ligne, position_intial_colone, partie_gagnee, partie_gagnee_pourcentage);
                    partie_gagnee = 0 ;    
                } 


            }


            else if( ( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) != 1 )&&( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) == 1 ) ){
                printf("sortir le checker avec le dé 2 et continuer avec le dé 1\n"); 
                }
            else if( ( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) == 1 )&&( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) != 1 ) ){
                printf("sortir le checker avec le dé 1 et continuer avec le dé 2\n"); 
                }


            }
        } 


        else if(couleur == 2){
            position_ligne_checker_out = 1; 
            position_colone_checker_out = 12; 
            if(tavli2[3][12] > 1){
                if( (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) == 1) && (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) == 1) ){
                    printf("Les deux dés peuvent être joués, peut importe l'ordre\n"); 
                }
                else if( (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) == 1) && (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) != 1) ){
                    printf("seul le premier dé est une possibilité\n"); 
                }
                else if( (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) != 1) && (function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) == 1) ){
                    printf("seul le deuxième dé est une possibilité\n"); 
                }
            }
            else{
                if( ( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) == 1 )&&( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) == 1 ) ){
                    printf("deux dés possibles pour sortir le checker\n"); 



                //on sort d'abord avec le checker 1 ; 
                for(int i = 0;i < 4; i++){
                    for(int j = 0; j < 13; j++){
                        tavli5[i][j] = tavli[i][j]; 
                    }
                }                    
                position_changer(tavli5, couleur, position_ligne_checker_out,position_colone_checker_out , dice1);

                taille_tableau = function_find__number_position_to_play(tavli5, couleur, dice1);                           
                int **position_possibles_checker_3 = malloc(sizeof(int *) * height); 
                for(int i = 0; i < height; i++){
                    position_possibles_checker_3[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                }

                for(int i = 0; i < 2; i++){
                    for(int j = 0; j < taille_tableau; j++){
                        position_possibles_checker_3[i][j] = 0; 
                    }
                }
                fonction_retourne_position_possible_pour_changement(tavli5, couleur, dice2, taille_tableau, position_possibles_checker_3);

                for(int m = 0 ; m < taille_tableau; m++){

                    for(int i = 0;i < 4; i++){
                        for(int j = 0; j < 13; j++){
                        tavli6[i][j] = tavli5[i][j]; 
                        }
                    }

                    //printf("tour numéro %d \n", m); 
                    position_intial_ligne = position_possibles_checker_3[0][m]; 
                    position_intial_colone = position_possibles_checker_3[1][m];
                    position_changer(tavli6, couleur, position_intial_ligne,position_intial_colone , dice2); 

                    for(int h = 0; h < nombre_essai; h++){
                        valeur_simulation = bot_simulation(tavli6, couleur); 
                        //printf("valeur simulation = %d et couleur = %d\n", valeur_simulation, couleur); 

                        if( valeur_simulation == 2){
                            partie_gagnee = partie_gagnee + 1; 
                        }
                    }     
                    partie_gagnee_pourcentage = (float)partie_gagnee/nombre_essai; 
                    printf("dé 1 (%d) :  ligne %d colone %d, dé 2 (%d) : ligne %d colone %d, score : %d, pourcentage = %f \n", dice1, position_ligne_checker_out, position_colone_checker_out, dice2,  position_intial_ligne, position_intial_colone, partie_gagnee, partie_gagnee_pourcentage);
                    partie_gagnee = 0 ;    
                }                 



                    // on sort d'abord avec le checker 2 ; 
                for(int i = 0;i < 4; i++){
                    for(int j = 0; j < 13; j++){
                        tavli7[i][j] = tavli[i][j]; 
                    }
                }                    
                position_changer(tavli7, couleur, position_ligne_checker_out,position_colone_checker_out , dice2);

                taille_tableau = function_find__number_position_to_play(tavli7, couleur, dice2);                           
                int **position_possibles_checker_4 = malloc(sizeof(int *) * height); 
                for(int i = 0; i < height; i++){
                    position_possibles_checker_4[i] = malloc(sizeof(unsigned int) * taille_tableau); 
                }

                for(int i = 0; i < 2; i++){
                    for(int j = 0; j < taille_tableau; j++){
                        position_possibles_checker_4[i][j] = 0; 
                    }
                }
                fonction_retourne_position_possible_pour_changement(tavli5, couleur, dice1, taille_tableau, position_possibles_checker_4);

                for(int m = 0 ; m < taille_tableau; m++){

                    for(int i = 0;i < 4; i++){
                        for(int j = 0; j < 13; j++){
                        tavli8[i][j] = tavli7[i][j]; 
                        }
                    }

                    //printf("tour numéro %d \n", m); 
                    position_intial_ligne = position_possibles_checker_4[0][m]; 
                    position_intial_colone = position_possibles_checker_4[1][m];
                    position_changer(tavli8, couleur, position_intial_ligne,position_intial_colone , dice1); 

                    for(int h = 0; h < nombre_essai; h++){
                        valeur_simulation = bot_simulation(tavli8, couleur); 
                        //printf("valeur simulation = %d et couleur = %d\n", valeur_simulation, couleur); 

                        if( valeur_simulation == 2){
                            partie_gagnee = partie_gagnee + 1; 
                        }
                    }     
                    partie_gagnee_pourcentage = (float)partie_gagnee/nombre_essai; 
                    printf("dé 1 (%d) :  ligne %d colone %d, dé 2 (%d) : ligne %d colone %d, score : %d, pourcentage = %f \n", dice2, position_ligne_checker_out, position_colone_checker_out, dice1,  position_intial_ligne, position_intial_colone, partie_gagnee, partie_gagnee_pourcentage);
                    partie_gagnee = 0 ;    
                } 


            }


            else if( ( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) != 1 )&&( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) == 1 ) ){
                printf("sortir le checker avec le dé 2 et continuer avec le dé 1\n"); 
                }
            else if( ( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice1 ) == 1 )&&( function_is_free(tavli2, couleur, position_ligne_checker_out, position_colone_checker_out, dice2 ) != 1 ) ){
                printf("sortir le checker avec le dé 1 et continuer avec le dé 2\n"); 
                }


            }
        } 

    }    


    else{
        //printf("rentre_ici\n"); 
        //commencer par le dé 1; 
        taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice1);                           
        int **position_possibles_dice_cas_1a = malloc(sizeof(int *) * height); 
        for(int i = 0; i < height; i++){
            position_possibles_dice_cas_1a[i] = malloc(sizeof(unsigned int) * taille_tableau); 
        }

        for(int i = 0; i < 2; i++){
            for(int j = 0; j < taille_tableau; j++){
                position_possibles_dice_cas_1a[i][j] = 0; 
            }
        }
        fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice1, taille_tableau, position_possibles_dice_cas_1a); 

        for(int m = 0 ; m < taille_tableau; m++){

            for(int i = 0;i < 4; i++){
                for(int j = 0; j < 13; j++){
                    tavli3[i][j] = tavli2[i][j]; 
                }
            }

            //printf("tour numéro %d \n", m); 
            position_intial_ligne = position_possibles_dice_cas_1a[0][m]; 
            position_intial_colone = position_possibles_dice_cas_1a[1][m];
            position_changer(tavli3, couleur, position_intial_ligne,position_intial_colone , dice1);
            //picture_generator(tavli3); 
            


            //dé numéro 2; 
            taille_tableau_2 = function_find__number_position_to_play(tavli3, couleur, dice2);                           
            int **position_possibles_dice_cas_1b = malloc(sizeof(int *) * height); 
            for(int i = 0; i < height; i++){
                position_possibles_dice_cas_1b[i] = malloc(sizeof(unsigned int) * taille_tableau_2); 
            }

            for(int i = 0; i < 2; i++){
                for(int j = 0; j < taille_tableau_2; j++){
                    position_possibles_dice_cas_1b[i][j] = 0; 
                }
            }
            fonction_retourne_position_possible_pour_changement(tavli3, couleur, dice2, taille_tableau_2, position_possibles_dice_cas_1b); 


            for(int n = 0; n < taille_tableau_2; n++){

                for(int i = 0;i < 4; i++){
                    for(int j = 0; j < 13; j++){
                        tavli4[i][j] = tavli3[i][j]; 
                    }
                }    
                position_intial_ligne_2 = position_possibles_dice_cas_1b[0][n]; 
                position_intial_colone_2 = position_possibles_dice_cas_1b[1][n];
                position_changer(tavli4, couleur, position_intial_ligne_2,position_intial_colone_2 , dice2);
                //picture_generator(tavli4); 

                //lancer le générateur un certain nombre de fois et mesurer les gains. 
                
                //printf("début de la simulation\n"); 
                for(int h = 0; h < nombre_essai; h++){

                    if(couleur == 1){
                        couleur = 2; 
                    }
                    else if(couleur == 2){
                        couleur = 1; 
                    }

                    valeur_simulation = bot_simulation(tavli4, couleur); 
                    //printf("valeur simulation = %d et couleur = %d\n", valeur_simulation, couleur); 


                    if( valeur_simulation != couleur){
                        partie_gagnee = partie_gagnee + 1; 
                    }

                    if(couleur ==1){
                        couleur = 2; 
                    }
                    else if(couleur == 2){
                        couleur = 1; 
                    }


                }

                partie_gagnee_pourcentage = (float)partie_gagnee/nombre_essai; 
                printf("dé 1 (%d) :  ligne %d colone %d, dé 2 (%d) : ligne %d colone %d, score : %d, pourcentage = %f \n", dice1, position_intial_ligne, position_intial_colone, dice2,  position_intial_ligne_2, position_intial_colone_2, partie_gagnee, partie_gagnee_pourcentage);
                partie_gagnee = 0 ; 
            }

            for (int i = 0; i < height; i++) {
                free(position_possibles_dice_cas_1b[i]);
            }
            // Free the array of pointers
            free(position_possibles_dice_cas_1b);

        }  

        for (int i = 0; i < height; i++) {
            free(position_possibles_dice_cas_1a[i]);
        }
        // Free the array of pointers
        free(position_possibles_dice_cas_1a);


        printf("\n\n"); 




        //commencer par le dé 2; 
        taille_tableau = function_find__number_position_to_play(tavli2, couleur, dice2);                           
        int **position_possibles_dice_cas_1c = malloc(sizeof(int *) * height); 
        for(int i = 0; i < height; i++){
            position_possibles_dice_cas_1c[i] = malloc(sizeof(unsigned int) * taille_tableau); 
        }

        for(int i = 0; i < 2; i++){
            for(int j = 0; j < taille_tableau; j++){
                position_possibles_dice_cas_1c[i][j] = 0; 
            }
        }
        fonction_retourne_position_possible_pour_changement(tavli2, couleur, dice2, taille_tableau, position_possibles_dice_cas_1c); 

        for(int m = 0 ; m < taille_tableau; m++){

            for(int i = 0;i < 4; i++){
                for(int j = 0; j < 13; j++){
                    tavli3[i][j] = tavli2[i][j]; 
                }
            }

            //printf("tour numéro %d \n", m); 
            position_intial_ligne = position_possibles_dice_cas_1c[0][m]; 
            position_intial_colone = position_possibles_dice_cas_1c[1][m];
            position_changer(tavli3, couleur, position_intial_ligne,position_intial_colone , dice2);
            //picture_generator(tavli3); 
            


            //dé numéro 2; 
            taille_tableau_2 = function_find__number_position_to_play(tavli3, couleur, dice1);                           
            int **position_possibles_dice_cas_1d = malloc(sizeof(int *) * height); 
            for(int i = 0; i < height; i++){
                position_possibles_dice_cas_1d[i] = malloc(sizeof(unsigned int) * taille_tableau_2); 
            }

            for(int i = 0; i < 2; i++){
                for(int j = 0; j < taille_tableau_2; j++){
                    position_possibles_dice_cas_1d[i][j] = 0; 
                }
            }
            fonction_retourne_position_possible_pour_changement(tavli3, couleur, dice1, taille_tableau_2, position_possibles_dice_cas_1d); 


            for(int n = 0; n < taille_tableau_2; n++){

                for(int i = 0;i < 4; i++){
                    for(int j = 0; j < 13; j++){
                        tavli4[i][j] = tavli3[i][j]; 
                    }
                }    
                position_intial_ligne_2 = position_possibles_dice_cas_1d[0][n]; 
                position_intial_colone_2 = position_possibles_dice_cas_1d[1][n];
                position_changer(tavli4, couleur, position_intial_ligne_2,position_intial_colone_2 , dice1);
                //picture_generator(tavli4); 

                //lancer le générateur un certain nombre de fois et mesurer les gains. 
                
                //printf("début de la simulation\n"); 
                for(int h = 0; h < nombre_essai; h++){

                    if(couleur == 1){
                        couleur = 2; 
                    }
                    else if(couleur == 2){
                        couleur = 1; 
                    }

                    valeur_simulation = bot_simulation(tavli4, couleur); 
                    //printf("valeur simulation = %d et couleur = %d\n", valeur_simulation, couleur); 


                    if( valeur_simulation != couleur){
                        partie_gagnee = partie_gagnee + 1; 
                    }

                    if(couleur ==1){
                        couleur = 2; 
                    }
                    else if(couleur == 2){
                        couleur = 1; 
                    }


                }

                partie_gagnee_pourcentage = (float)partie_gagnee/nombre_essai; 
                printf("dé 2 (%d) :  ligne %d colone %d, dé 1 (%d) : ligne %d colone %d, score : %d, pourcentage = %f \n", dice2, position_intial_ligne, position_intial_colone, dice1,  position_intial_ligne_2, position_intial_colone_2, partie_gagnee, partie_gagnee_pourcentage);
                partie_gagnee = 0 ; 
            }

            for (int i = 0; i < height; i++) {
                free(position_possibles_dice_cas_1d[i]);
            }
            // Free the array of pointers
            free(position_possibles_dice_cas_1d);

        }  

        for (int i = 0; i < height; i++) {
            free(position_possibles_dice_cas_1c[i]);
        }
        // Free the array of pointers
        free(position_possibles_dice_cas_1c);                    

    }


    
}

int main(){

    srand(time(0));
    int tavli[4][13]; 
    initialisation_tavli(tavli); 
    picture_generator(tavli);
    int couleur; 
    int de1; 
    int de2; 
    int partie_finie = 1; 
    int simulation_valeur; 
    int position_ligne; 
    int position_colone; 
    int nombre_de_jouable; 

    printf("Start of the simulation\n"); 

    while(partie_finie == 1){
        printf("Give the current color\n");
        scanf("%d", &couleur); 
        printf("Give value of dice 1\n"); 
        scanf("%d", &de1); 
        printf("give value of dice 2\n"); 
        scanf("%d", &de2); 

        printf("to ask for a simulation, press 1, for no simulation, press 0\n"); 
        scanf("%d", &simulation_valeur); 

        if(simulation_valeur == 1){
            fonction_simulation_multiple(tavli, couleur, de1, de2); 

            printf("Insert the number of possibles dices to play\n"); 
            scanf("%d", &nombre_de_jouable); 

            if(nombre_de_jouable == 1){
                printf("Insert the dice chosen for the change of position\n"); 
                scanf("%d", &de1); 
                printf("Insert the line of the position to change\n"); 
                scanf("%d", &position_ligne); 
                printf("Insert the column of the position to change\n"); 
                scanf("%d", &position_colone); 
                position_changer(tavli, couleur, position_ligne,position_colone , de1); 
                picture_generator(tavli); 

            }

            else if(nombre_de_jouable == 2){

                printf("Insert the dice chosen for the change of position\n"); 
                scanf("%d", &de1); 
                printf("Insert the line of the position to change\n"); 
                scanf("%d", &position_ligne); 
                printf("Insert the column of the position to change\n"); 
                scanf("%d", &position_colone); 
                position_changer(tavli, couleur, position_ligne,position_colone , de1); 
                picture_generator(tavli);

                printf("Insert the dice chosen for the change of position\n"); 
                scanf("%d", &de2); 
                printf("Insert the line of the position to change\n"); 
                scanf("%d", &position_ligne); 
                printf("Insert the column of the position to change\n"); 
                scanf("%d", &position_colone); 
                position_changer(tavli, couleur, position_ligne,position_colone , de2);    
                picture_generator(tavli);            

            }
            
            else{

            }
        }

        else{

            printf("Insert the number of possibles dices to play\n"); 
            scanf("%d", &nombre_de_jouable); 

            if(nombre_de_jouable == 1){
                printf("Insert the dice chosen for the change of position\n"); 
                scanf("%d", &de1); 
                printf("Insert the line of the position to change\n"); 
                scanf("%d", &position_ligne); 
                printf("Insert the column of the position to change\n"); 
                scanf("%d", &position_colone); 
                position_changer(tavli, couleur, position_ligne,position_colone , de1); 
                picture_generator(tavli); 

            }

            else if(nombre_de_jouable == 2){
                printf("Insert the dice chosen for the change of position\n"); 
                scanf("%d", &de1); 
                printf("Insert the line of the position to change\n"); 
                scanf("%d", &position_ligne); 
                printf("Insert the column of the position to change\n"); 
                scanf("%d", &position_colone); 
                position_changer(tavli, couleur, position_ligne,position_colone , de1); 
                picture_generator(tavli);

                printf("Insert the dice chosen for the change of position\n"); 
                scanf("%d", &de2); 
                printf("Insert the line of the position to change\n"); 
                scanf("%d", &position_ligne); 
                printf("Insert the column of the position to change\n"); 
                scanf("%d", &position_colone); 
                position_changer(tavli, couleur, position_ligne,position_colone , de2);
                picture_generator(tavli);                
            }
            
            else{

            }
        }

        printf("Enter 1 is the game is not over, otherwise insert 0\n"); 
        scanf("%d", &partie_finie); 
    }




    return 0; 
}