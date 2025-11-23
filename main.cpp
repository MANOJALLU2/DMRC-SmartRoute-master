#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <cstdlib>
#include <unordered_map>
#include <bits/stdc++.h>
#include "UI.h"
using namespace std;

#include <chrono>
#include <iomanip>
int timetaken(float dist)
{
    float speed=0.55;
    return ceil(dist/speed);
}

bool isWeekend() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto now_time_t = system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_time_t);
    int day_of_week = local_tm.tm_wday;
    return (day_of_week == 0 || day_of_week == 6);
}

float GetPrice(float distance){
    if (isWeekend()){
        if (distance<2){return 10;}
        if (distance>=2 && distance<5){return 10;}
        if (distance>=5 && distance<12){return 20;}
        if (distance>=12 && distance<21){return 30;}
        if (distance>=21 && distance<32){return 40;}
        if (distance>=32){return 50;}
    }else{
        if (distance<2){return 10;}
        if (distance>=2 && distance<5){return 20;}
        if (distance>=5 && distance<12){return 30;}
        if (distance>=12 && distance<21){return 40;}
        if (distance>=21 && distance<32){return 50;}
        if (distance>=32){return 60;}
    }
    return 0;
}

void generateAndViewGraph(const std::string& dotFilename, const std::string& outputFilename) {
    std::string command = "dot -Tpng " + dotFilename + " -o " + outputFilename;
    int result = system(command.c_str());
    if (result != 0) {
        std::cerr << "Error: Could not generate graph image from DOT file." << std::endl;
        return;
    }
    std::string viewCommand = outputFilename;
}
void generateAndViewGraph1(const std::string& dotFilename, const std::string& outputFilename) {
    std::string command = "dot -Tpng " + dotFilename + " -o " + outputFilename;
    int result = system(command.c_str());
    if (result != 0) {
        std::cerr << "Error: Could not generate graph image from DOT file." << std::endl;
        return;
    }
    std::string viewCommand = outputFilename;
    result = system(viewCommand.c_str());
    if (result != 0) {
        std::cerr << "Error: Could not open graph image file." << std::endl;
    }
}


int countLines(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Could not open file " << filename << endl;
        return -1;
    }

    string line;
    int lineCount = 0;

    while (getline(file, line)) {
        ++lineCount;
    }

    file.close();
    return lineCount;
}

struct Station{
    string StationName;
    string Color;
    int StationCode;
};

class Network {
public:
    int StationsCount;
    vector<struct Station> Stations;
    vector<vector<pair<int, float>>> NetworkStructure;
    unordered_map<string,int> mp;
public:
    Network(int StationsCount) {
        this->StationsCount = StationsCount;
        this->Stations = vector<struct Station>(StationsCount);
        this->NetworkStructure = vector<vector<pair<int, float>>>(StationsCount, vector<pair<int, float>>(0));
    }

    void addStation(string Name, string Color, int StationCode) {
        if (StationCode >= 0 && StationCode < StationsCount) {
            this->Stations[StationCode].StationName = Name;
            this->Stations[StationCode].Color = Color;
            this->Stations[StationCode].StationCode = StationCode;
            this->mp[Name] = StationCode;
        } else {
            cerr << "Error: Station code out of range: " << StationCode << endl;
        }
    }

    void addRoute(int StationCodeStart, int StationCodeDestination, float Distance) {
        if (StationCodeStart >= 0 && StationCodeStart < StationsCount && 
            StationCodeDestination >= 0 && StationCodeDestination < StationsCount) {
            this->NetworkStructure[StationCodeStart].push_back({StationCodeDestination, Distance});
        } else {
            cerr << "Error: Route with invalid station codes: " 
                 << StationCodeStart << " to " << StationCodeDestination << endl;
        }
    }



    void generateDotFile(const string& filename) {
        ofstream outFile(filename);
        if (!outFile.is_open()) {
            cerr << "Error: Could not open file " << filename << endl;
            return;
        }

        outFile << "digraph DMRC {\n";
        outFile << "rankdir=LR;\n";
        outFile << "graph [bgcolor=white];\n";

        for (int i = 0; i < this->StationsCount; ++i) {
            if (this->Stations[i].Color!="VOILET"){
            outFile << "  " << this->Stations[i].StationCode 
                    << " [label=\"" << this->Stations[i].StationName 
                    << "\" color=\"" << this->Stations[i].Color << "\"];\n";
            }else{
                outFile << "  " << this->Stations[i].StationCode 
                    << " [label=\"" << this->Stations[i].StationName 
                    << "\" color=\"" << "BLACK" << "\"];\n";
            }

            for (const auto& route : this->NetworkStructure[i]) {
                outFile << "  " << i << " -> " << route.first 
                        << " [label=\"" << route.second << "\"];\n";
            }
        }
        outFile << "}\n";
        outFile.close();
    }

    int dijkstras(string StationStartName,string StationEndName){
        int StationStartCode = mp[StationStartName];
        int StationEndCode = mp[StationEndName];
        vector<int> visited(this->StationsCount,1e9);
        vector<int> prev(this->StationsCount,-2);
        priority_queue<pair<float, int>, vector<pair<float, int>>, greater<pair<float, int>>> pq;
        pq.push({0,StationStartCode});
        visited[StationStartCode] = 0;
        prev[StationStartCode] = -1;
        float min_distance = 1e9;
        while (!pq.empty()){
            pair<float,int> top = pq.top();
            pq.pop();
            for (auto &i: this->NetworkStructure[top.second]){
                if (visited[i.first]==1e9){
                    visited[i.first]=top.first+i.second;
                    prev[i.first] = top.second;
                    float dist = top.first+i.second;
                    pq.push({dist,i.first});
                    if (i.first==StationEndCode){
                        min_distance = min(min_distance,dist);
                        while (!pq.empty()){pq.pop();}
                        break;
                    }
                }
            }

        }
        vector<int> path;
        path.push_back(StationEndCode);
        while (true){
            int temp = prev[path[path.size()-1]];
            if (temp!=-1){
                path.push_back(temp);
            }else{
                break;
            }
        }
        float min_fare = GetPrice(min_distance);
        int t1 = PrintPath(path,min_distance,min_fare,timetaken(min_distance));
        generateDijkstraDotFile(path,"dmrcdijkstra.dot");
        generateAndViewGraph("dmrcdijkstra.dot", "dmrcdijkstra.png");
        return t1;

    }
    pair<int,int> dijkstras1(string StationStartName,string StationEndName,int x,int y){
        int StationStartCode = mp[StationStartName];
        int StationEndCode = mp[StationEndName];
        vector<int> visited(this->StationsCount,1e9);
        vector<int> prev(this->StationsCount,-2);
        priority_queue<pair<float, int>, vector<pair<float, int>>, greater<pair<float, int>>> pq;
        pq.push({0,StationStartCode});
        visited[StationStartCode] = 0;
        prev[StationStartCode] = -1;
        float min_distance = 1e9;
        while (!pq.empty()){
            pair<float,int> top = pq.top();
            pq.pop();
            for (auto &i: this->NetworkStructure[top.second]){
                if (visited[i.first]==1e9){
                    visited[i.first]=top.first+i.second;
                    prev[i.first] = top.second;
                    float dist = top.first+i.second;
                    pq.push({dist,i.first});
                    if (i.first==StationEndCode){
                        min_distance = min(min_distance,dist);
                        while (!pq.empty()){pq.pop();}
                        break;
                    }
                }
            }

        }
        vector<int> path;
        path.push_back(StationEndCode);
        while (true){
            int temp = prev[path[path.size()-1]];
            if (temp!=-1){
                path.push_back(temp);
            }else{
                break;
            }
        }
        float min_fare = GetPrice(min_distance);
        int t1 = PrintPath(path,min_distance,min_fare,timetaken(min_distance),x,y);
        generateDijkstraDotFile(path,"dmrcdijkstra.dot");
        generateAndViewGraph("dmrcdijkstra.dot", "dmrcdijkstra.png");
        return {t1,min_distance};

    }

    void SetColor(string color) {
        HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
        if (color == "BLUE") {
            SetConsoleTextAttribute(hConsole, FOREGROUND_BLUE | FOREGROUND_INTENSITY);
        } else if (color == "MAGENTA") {
            SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
        } else if (color == "YELLOW") {
            SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY);
        } else if (color == "GREEN") {
            SetConsoleTextAttribute(hConsole, FOREGROUND_GREEN | FOREGROUND_INTENSITY);
        } else if (color == "ORANGE") {
            SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_GREEN);
        } else if (color == "RED") {
            SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_INTENSITY);
        } else if (color == "PINK") {
            SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_BLUE);
        } else if (color == "VIOLET") {
            SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_BLUE | BACKGROUND_GREEN);
        } else if (color == "AQUA") {
            SetConsoleTextAttribute(hConsole, FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
        } else {
            SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE); // Default white color
        }
    }
    int PrintPath(vector<int> &path,float min_distance,float min_fare,int time_taken) {
        int boxWidth = 153;
        int currentLength = 0;
        int currentLine = 9;

        int indent = 16;

        gotoxy(indent, currentLine+2);
        currentLine+=2;
        cout<<"THE PATH WITH THE SHORTEST FARE & DISTANCE IS:"<<endl;

        string prevcolor = this->Stations[path[path.size() - 1]].Color;
        string segment = this->Stations[path[path.size() - 1]].StationName + " ===> ";
        SetColor(prevcolor);
        gotoxy(indent, currentLine+2);
        currentLine+=2;
        delay(160);
        cout << segment;
        currentLength += segment.length();

        for (int i = path.size() - 2; i > 0; i--) {
            if (this->Stations[path[i]].Color != prevcolor && this->Stations[path[i - 1]].Color != prevcolor) {
                cout << endl;
                currentLength = 0;
                gotoxy(indent, currentLine+2);
                currentLine+=2;
                SetColor("DEFAULT");
                delay(160);
                cout<<"(Change from ";
                SetColor(prevcolor);
                delay(160);
                cout<<prevcolor + " line to ";
                SetColor(this->Stations[path[i]].Color);
                delay(160);
                cout<<this->Stations[path[i]].Color + " line.";
                SetColor("DEFAULT");
                delay(160);
                cout<<") "<<endl;
                SetColor(this->Stations[path[i]].Color);
                // cout << segment << endl;
                gotoxy(indent, currentLine+2);
                currentLine+=2;
                prevcolor = this->Stations[path[i]].Color;
                currentLength = segment.length();
                segment = this->Stations[path[i]].StationName + " ===> ";
                delay(160);
                cout << segment;
                currentLength += segment.length();
            } else {
                segment = this->Stations[path[i]].StationName + " ===> ";
                if (currentLength + segment.length() > (boxWidth-indent-6)) {
                    delay(160);
                    cout << endl;
                    gotoxy(indent, currentLine+2);
                    currentLine+=2;
                    currentLength = 0;
                }
                SetColor(prevcolor);
                delay(160);
                cout << segment;
                currentLength += segment.length();
            }
        }

        segment = this->Stations[path[0]].StationName;
        if (currentLength + segment.length() > (boxWidth-indent-6)) {
            gotoxy(indent, currentLine+2);
            currentLine+=2;
            delay(160);
            cout << endl;
        }
        SetColor(prevcolor);
        delay(160);
        cout << segment << endl;
        gotoxy(indent, currentLine+2);
        currentLine+=2;
        delay(160);
        cout << "(Reached Your Destination)" << endl;
        SetColor("Default");
        gotoxy(indent, currentLine+2);
        currentLine+=2;
        delay(160);
        cout<<"Min Distance :"<<min_distance<<" KMS"<<endl;
        gotoxy(indent, currentLine+2);
        currentLine+=2;
        delay(160);
        cout<<"Min Fare :"<<min_fare<<" RS"<<endl;
        gotoxy(indent, currentLine+2);
        currentLine+=2;
        delay(160);
        cout<<"Time Taken :"<<time_taken<<" MIN"<<endl;
        gotoxy(indent, currentLine+2);
        currentLine+=2;
        delay(160);
        cout<<"No Of Intermediate Stations :"<<path.size()-2<<endl;
        return currentLine+1;
    }
    int PrintPath(vector<int> &path,float min_distance,float min_fare,int time_taken,int x,int y) {
        int boxWidth = 153;
        int currentLength = 0;
        int currentLine = y;

        int indent = 0;

        gotoxy(indent, currentLine+2);
        currentLine+=2;
        cout<<"THE PATH WITH THE SHORTEST FARE & DISTANCE IS:"<<endl;

        string prevcolor = this->Stations[path[path.size() - 1]].Color;
        string segment = this->Stations[path[path.size() - 1]].StationName + " ===> ";
        SetColor(prevcolor);
        gotoxy(indent, currentLine+2);
        currentLine+=2;
        delay(160);
        cout << segment;
        currentLength += segment.length();

        for (int i = path.size() - 2; i > 0; i--) {
            if (this->Stations[path[i]].Color != prevcolor && this->Stations[path[i - 1]].Color != prevcolor) {
                cout << endl;
                currentLength = 0;
                gotoxy(indent, currentLine+2);
                currentLine+=1;
                SetColor("DEFAULT");
                delay(160);
                cout<<"(Change from ";
                SetColor(prevcolor);
                delay(160);
                cout<<prevcolor + " line to ";
                SetColor(this->Stations[path[i]].Color);
                delay(160);
                cout<<this->Stations[path[i]].Color + " line.";
                SetColor("DEFAULT");
                delay(160);
                cout<<") "<<endl;
                SetColor(this->Stations[path[i]].Color);
                // cout << segment << endl;
                gotoxy(indent, currentLine+2);
                currentLine+=1;
                prevcolor = this->Stations[path[i]].Color;
                currentLength = segment.length();
                segment = this->Stations[path[i]].StationName + " ===> ";
                delay(160);
                cout << segment;
                currentLength += segment.length();
            } else {
                segment = this->Stations[path[i]].StationName + " ===> ";
                if (currentLength + segment.length() > (boxWidth-indent-6)) {
                    delay(160);
                    cout << endl;
                    gotoxy(indent, currentLine+2);
                    currentLine+=1;
                    currentLength = 0;
                }
                SetColor(prevcolor);
                delay(160);
                cout << segment;
                currentLength += segment.length();
            }
        }

        segment = this->Stations[path[0]].StationName;
        if (currentLength + segment.length() > (boxWidth-indent-6)) {
            gotoxy(indent, currentLine+2);
            currentLine+=1;
            delay(160);
            cout << endl;
        }
        SetColor(prevcolor);
        delay(160);
        cout << segment << endl;
        gotoxy(indent, currentLine+2);
        currentLine+=1;
        delay(160);
        cout << "(Reached Your Destination)" << endl;
        SetColor("Default");
        gotoxy(indent, currentLine+2);
        currentLine+=1;
        delay(160);
        cout<<"Min Distance :"<<min_distance<<" KMS"<<endl;
        gotoxy(indent, currentLine+2);
        currentLine+=1;
        delay(160);
        cout<<"Min Fare :"<<min_fare<<" RS"<<endl;
        gotoxy(indent, currentLine+2);
        currentLine+=1;
        delay(160);
        cout<<"Time Taken :"<<time_taken<<" MIN"<<endl;
        gotoxy(indent, currentLine+2);
        currentLine+=1;
        delay(160);
        cout<<"No Of Intermediate Stations :"<<path.size()-2<<endl;
        return currentLine+1;
    }

    void generateDijkstraDotFile(const vector<int>& path, const string& filename) {
        ofstream outFile(filename);
        if (!outFile.is_open()) {
            cerr << "Error: Could not open file " << filename << endl;
            return;
        }

        outFile << "digraph DijkstraPath {\n";
        outFile << "rankdir=LR;\n";
        outFile << "graph [bgcolor=white];\n";
        for (int i = 0; i < path.size(); ++i) {
            int j = path[i];
            if (Stations[j].Color != "VOILET"){
            outFile << "  " << j << " [label=\"" << Stations[j].StationName 
                    << "\" color=\"" << Stations[j].Color << "\"];\n";
            }else{
                outFile << "  " << j << " [label=\"" << Stations[j].StationName 
                    << "\" color=\"" << "BLACK" << "\"];\n";
            }
        }

        for (int i = 0; i < path.size() - 1; ++i) {
            int u = path[i];
            int v = path[i + 1];
            float distance = 0.0;
            for (const auto& edge : NetworkStructure[v]) {
                if (edge.first == u) {
                    distance = edge.second;
                    break;
                }
            }
            outFile << "  " << v << " -> " << u 
                    << " [label=\"" << distance << "\" color=\"red\"];\n";
        }

        outFile << "}\n";
        outFile.close();
    }
    vector<vector<float>> computeShortestPaths() {
        vector<vector<float>> dist(StationsCount, vector<float>(StationsCount, 1e9));
        for (int i = 0; i < StationsCount; ++i) {
            dist[i][i] = 0;
            for (const auto& route : NetworkStructure[i]) {
                dist[i][route.first] = route.second;
            }
        }

        for (int k = 0; k < StationsCount; ++k) {
            for (int i = 0; i < StationsCount; ++i) {
                for (int j = 0; j < StationsCount; ++j) {
                    if (dist[i][k] + dist[k][j] < dist[i][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }

        return dist;
    }
    void PrintStations(int x, int y){
        const int BOX_WIDTH = 153;
        const int RIGHT_INDENT = 6;
        const int START_X = x;
        const int START_Y = y;
        int currentX = START_X;
        int currentY = START_Y;
        int indent = 0;

        gotoxy(currentX, currentY);
        currentY++;
        indent = START_X;
        for (auto &i: this->Stations){
            if (currentX + indent + i.StationName.length() > BOX_WIDTH - RIGHT_INDENT) {
                currentY++;
                currentX = START_X;
                gotoxy(currentX, currentY);
            }
            SetColor(i.Color);
            cout << i.StationName << " ";
            currentX += i.StationName.length() + 1;
        }
    }
    vector<int> solveTSP(vector<int> stations) {
            int n = stations.size();
            vector<vector<float>> dist = computeShortestPaths();
            vector<int> path(n);
            iota(path.begin(), path.end(), 0); // Generate the initial path

            float minCost = FLT_MAX;
            vector<int> bestPath;

            do {
                float cost = 0;
                for (int i = 0; i < n - 1; ++i) {
                    cost += dist[stations[path[i]]][stations[path[i + 1]]];
                }
                cost += dist[stations[path[n - 1]]][stations[path[0]]]; // Return to start

                if (cost < minCost) {
                    minCost = cost;
                    bestPath = path;
                }
            } while (next_permutation(path.begin(), path.end()));

            vector<int> result;
            for (int i : bestPath) {
                result.push_back(stations[i]);
            }
            return result;
        }
        vector<int> greedyTSP(const vector<int>& stations) {
            int n = stations.size();
            if (n == 0) return {}; // Handle empty input case

            vector<vector<float>> dist = computeShortestPaths();
            
            // Verify the distance matrix size
            if (dist.size() != dist[0].size()) {
                cerr << "Error: Distance matrix is not square" << endl;
                return {};
            }

            // Verify that the indices in 'stations' are valid
            for (int i=0;i<n;i++){
                
            // for (int station : stations) {
            int station = stations[i];
                if (station >= dist.size() || station >= dist[0].size()) {
                    cerr << "Error: Station index out of bounds in distance matrix" << endl;
                    return {};
                }
            }

            vector<int> path;
            vector<bool> visited(n, false);
            float totalCost = 0;

            int current = 0; // Start from the first station
            path.push_back(stations[current]);
            visited[current] = true;

            for (int i = 1; i < n; ++i) {
                float minDist = FLT_MAX;
                int next = -1;

                for (int j = 0; j < n; ++j) {
                    if (!visited[j] && dist[stations[current]][stations[j]] < minDist) {
                        minDist = dist[stations[current]][stations[j]];
                        next = j;
                    }
                }

                if (next != -1) {
                    path.push_back(stations[next]);
                    visited[next] = true;
                    totalCost += minDist;
                    current = next;
                } else {
                    cerr << "Error: No unvisited node found" << endl;
                    return {}; // Handle the error as needed
                }
            }

            // Return to the start
            // totalCost += dist[stations[current]][stations[path[0]]];
            path.push_back(stations[0]);

            // For debugging
            // cout << "Total cost: " << totalCost << endl;

            return path;
        }
    

};

int main() {
    system("cls");
    std::ifstream InFile("Routes.txt");
    std::ifstream InFileStations("StationCodes.txt");
    std::ifstream InFileColors("StationColors.txt");

    if (!InFile.is_open() || !InFileStations.is_open() || !InFileColors.is_open()) {
        cerr << "Error: Could not open one or more input files." << endl;
        return 1;
    }

    int stations = countLines("Routes.txt");
    if (stations <= 0) {
        cerr << "Error: Invalid number of stations." << endl;
        return 1;
    }
    
    Network DMRC(stations);

    for (int station = 0; station < stations; ++station) {
        int noofroutes;
        int Code;
        string StationCode;
        string StationColor;

        if (!(InFile >> noofroutes >> Code)) {
            cerr << "Error: Failed to read number of routes or station code." << endl;
            return 1;
        }

        getline(InFileStations, StationCode);
        if (StationCode.empty()) {
            cerr << "Error: Failed to read station name." << endl;
            return 1;
        }

        if (!(InFileColors >> StationColor)) {
            cerr << "Error: Failed to read station color." << endl;
            return 1;
        }
        DMRC.addStation(StationCode, StationColor, Code-1);

        for (int outStation = 0; outStation < noofroutes; ++outStation) {
            int OutStationCode;
            float Distance;

            if (!(InFile >> OutStationCode >> Distance)) {
                cerr << "Error: Failed to read route data." << endl;
                return 1;
            }

            DMRC.addRoute(Code-1, OutStationCode-1, Distance);
        }
    }
    while (true){
        system("cls");
        int mode = UI1();
        if (mode == 1){
            string startdij;
            string enddij;
            gotoxy(65,2);
            cout<<"++SHORTEST PATH FINDING IN DMRC METRO++"<<endl;
            gotoxy(16,4);
            cout<<"Enter the Starting Station Name:";
            getline(cin, startdij);
            cout<<endl;
            gotoxy(16,6);
            cout<<"Enter the Ending Station Name:";
            getline(cin, enddij);
            cout<<endl;
            int temp = DMRC.dijkstras(startdij, enddij);
            gotoxy(65, temp + 1);
            cout<<"PRESS ENTER TO CONTINUE";
            char t2 = getchar();
        } else if (mode == 2){
            gotoxy(65,2);
            cout<<"++LIST OF ALL STATIONS IN DMRC METRO++"<<endl;
            gotoxy(20,6);
            DMRC.PrintStations(20,6);
            cout<<"PRESS ENTER TO CONTINUE";
            char t2 = getchar();
        } else if (mode == 3){
            gotoxy(64,2);
            cout<<"OPENING DMRC MAP..."<<endl;
            generateAndViewGraph1("dmrc.dot", "dmrc.png");
        } else if (mode == 4){
            gotoxy(65,2);
            cout<<"++TRAVEL PLANNER++"<<endl;
            gotoxy(16,4);
            cout<<"Enter the names of all stations you want to visit, separated by commas:"<<endl;
            gotoxy(16,6);
            string stationsInput;
            getline(cin, stationsInput);
            vector<string> stationNames;
            stringstream ss(stationsInput);
            string station;
            while (getline(ss, station, ',')) {
                stationNames.push_back(station);
            }

            vector<int> stationCodes;
            for (const auto& name : stationNames) {
                if (DMRC.mp.find(name) != DMRC.mp.end()) {
                    stationCodes.push_back(DMRC.mp[name]);
                } else {
                    cerr << "Error: Station " << name << " not found." << endl;
                    return 1;
                }
            }

            // Apply TSP
            vector<int> tspPath = DMRC.solveTSP(stationCodes);
            // For each path in tspPath, use dijkstra to find and print the shortest path
            float totalDistance = 0;
            float totalFare = 0;
            int x = 16;
            int y = 8;
            for (size_t i = 0; i < tspPath.size() - 1; ++i) {
                int startStation = tspPath[i];
                int endStation = tspPath[i + 1];
                pair<int,int> temp = DMRC.dijkstras1(DMRC.Stations[startStation].StationName, DMRC.Stations[endStation].StationName,x,y);
                totalDistance += temp.second;
                totalFare += GetPrice(temp.second);
                y = temp.first+3;
                cout<<endl;
                cout<<"++ ANOTHER JOURNEY STARTS ++"<<endl;
                cout<<endl;
            }
            int startStation = tspPath[tspPath.size()-1];
                int endStation = tspPath[0];
                pair<int,int> temp = DMRC.dijkstras1(DMRC.Stations[startStation].StationName, DMRC.Stations[endStation].StationName,x,y);
                totalDistance += temp.second;
                totalFare += GetPrice(temp.second);
                y = temp.first+3;
                cout<<endl;
                // cout<<"++ ANOTHER JOURNEY STARTS ++"<<endl;
                // cout<<endl;
            gotoxy(65, y);
            cout << "Total Distance: " << totalDistance << " KMS" << endl;
            cout << "Total Fare: " << totalFare << " RS" << endl;
            cout << "PRESS ENTER TO CONTINUE";
            char t2 = getchar();
        } else if (mode == 5){
            gotoxy(65,2);
            cout<<"++TRAVEL PLANNER++"<<endl;
            gotoxy(16,4);
            cout<<"Enter the names of all stations you want to visit, separated by commas:"<<endl;
            gotoxy(16,6);
            string stationsInput;
            getline(cin, stationsInput);
            vector<string> stationNames;
            stringstream ss(stationsInput);
            string station;
            while (getline(ss, station, ',')) {
                stationNames.push_back(station);
            }

            vector<int> stationCodes;
            for (const auto& name : stationNames) {
                if (DMRC.mp.find(name) != DMRC.mp.end()) {
                    stationCodes.push_back(DMRC.mp[name]);
                } else {
                    cerr << "Error: Station " << name << " not found." << endl;
                    return 1;
                }
            }

            // Apply TSP
            vector<int> tspPath = DMRC.greedyTSP(stationCodes);
            // For each path in tspPath, use dijkstra to find and print the shortest path
            float totalDistance = 0;
            float totalFare = 0;
            int x = 16;
            int y = 8;
            for (size_t i = 0; i < tspPath.size() - 1; ++i) {
                int startStation = tspPath[i];
                int endStation = tspPath[i + 1];
                pair<int,int> temp = DMRC.dijkstras1(DMRC.Stations[startStation].StationName, DMRC.Stations[endStation].StationName,x,y);
                totalDistance += temp.second;
                totalFare += GetPrice(temp.second);
                y = temp.first+3;
                cout<<endl;
                cout<<"++ ANOTHER JOURNEY STARTS ++"<<endl;
                cout<<endl;
            }
            gotoxy(65, y);
            cout << "Total Distance: " << totalDistance << " KMS" << endl;
            cout << "Total Fare: " << totalFare << " RS" << endl;
            cout << "PRESS ENTER TO CONTINUE";
            char t2 = getchar();
        }
        
        else {
            gotoxy(65,80);
            cout<<"EXITING...THANK YOU FOR VISITING"<<endl;
            break;
        }
    }


    InFile.close();
    InFileStations.close();
    InFileColors.close();
    std::string dotFilename = "dmrc.dot";
    std::string outputFilename = "dmrc.png";
    DMRC.generateDotFile("dmrc.dot");
    generateAndViewGraph(dotFilename, outputFilename);
    gotoxy(16, 80);
    return 0;
}
