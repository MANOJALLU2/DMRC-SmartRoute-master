#include <iostream>
#include <windows.h>
#include <string>
#include <conio.h>  // For getch()
using namespace std;

void gotoxy(int x, int y);
void clrscreen();
void delay(unsigned int ms);
void drawbox(int x1, int y1, int x2, int y2, int d);
void logo(int x, int y);
void UI();

COORD coord;

void gotoxy(int x, int y)
{
    coord.X = x;
    coord.Y = y;
    SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
}

void clrscreen()
{
    system("cls");
    drawbox(1, 0, 153, 0, 0);
}

void delay(unsigned int ms)
{
    clock_t goal = ms + clock();
    while (goal > clock());
}

void drawbox(int x1, int y1, int x2, int y2, int d)
{
    gotoxy(x1, y1);
    cout << "+";
    delay(d);
    gotoxy(x2, y2);
    cout << "+";
    delay(d);
    int i;
    for (i = 1; i < x2 - x1; i++)
    {
        gotoxy(x1 + i, y1);
        cout << "-";
        delay(d);
        gotoxy(x2 - i, y2);
        cout << "-";
        delay(d);
    }
    gotoxy(x1, y2);
    cout << "+";
    gotoxy(x2, y1);
    cout << "+";
    int j;
    for (j = 1; j < y2 - y1; j++)
    {
        gotoxy(x2, y1 + j);
        cout << "|";
        delay(d);
        gotoxy(x1, y2 - j);
        cout << "|";
        delay(d);
    }
}

void logo(int x, int y)
{
    gotoxy(x, y);
    cout << "   ___                     ___       ____";
    gotoxy(x, y + 1);
    cout << " ||   \\\\   ||\\\\    //||  ||   \\\\   //    \\\\";
    gotoxy(x, y + 2);
    cout << " ||    ||  || \\\\  // ||  ||    || ||";
    gotoxy(x, y + 3);
    cout << " ||    ||  ||  \\\\//  ||  ||___//  ||";
    gotoxy(x, y + 4);
    cout << " ||    ||  ||        ||  ||  \\\\   ||";
    gotoxy(x, y + 5);
    cout << " ||___//   ||        ||  ||   \\\\   \\\\____//";
}
void printWithIndentation(const string& text, int startX, int startY, int boxWidth, int indent) {
    int currentX = startX;
    int currentY = startY;
    int currentLength = 0;

    vector<string> words;
    stringstream ss(text);
    string word;

    while (ss >> word) {
        words.push_back(word);
    }

    for (const auto& word : words) {
        int wordLength = word.length() + 1; // +1 for space

        if (currentX + currentLength + wordLength > boxWidth - indent) {
            cout << endl;
            gotoxy(startX, ++currentY); // Move to next line
            currentX = startX;
            currentLength = 0;
        }

        cout << word << " ";
        currentLength += wordLength;
    }
    cout << endl;
}

int UI1()
{
    system("color 0A");
    drawbox(1, 0, 153, 0, 4);
    system("color 0C");
    delay(90);
    system("color 0A");
    delay(90);
    logo(60, 4);
    system("color 0C");
    delay(90);
    drawbox(53, 3, 110, 11, 4);
    drawbox(51, 2, 112, 12, 4);
    system("color 0C");
    delay(90);
    system("color 0A");
    delay(90);
    gotoxy(65, 14);
    cout << "WELCOME TO DELHI METRO DESKTOP APP";
    system("color 0C");
    delay(90);
    system("color 0D");
    delay(90);
    gotoxy(70, 18);
    cout<<"SELECT THE SERVICE BELOW"<<endl;
    gotoxy(50,21);
    cout<<"1. SHORTEST ROUTE FROM ONE PLACE TO OTHER"<<endl;
    gotoxy(50,23);
    cout<<"2. DISPLAY ALL STATIONS"<<endl;
    gotoxy(50,25);
    cout<<"3. OPEN DMRC MAP"<<endl;
    gotoxy(50,27);
    cout<<"4. TRAVEL PLANNER Using Branch & Bound"<<endl;
    gotoxy(50,29);
    cout<<"5. TRAVEL PLANNER Using Greedy"<<endl;
    gotoxy(50,31);
    cout<<"6. EXIT"<<endl;
    gotoxy(50,33);
    cout << "PRESS NUMBER TO CONTINUE";
    gotoxy(55, 33);
    char ch = getch();  // Use getch() to capture the enter key without echoing

    // Clear the screen and take input
    clrscreen();
    gotoxy(16, 3);
    return ch-'0';
}


