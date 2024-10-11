#include <iostream>
#include <vector> 
#include <algorithm>
#include <string> 
using namespace std;
#define ll long long
void moveDisk (ll numberofdisk, vector<vector <ll>> &moves, ll sourceStack, ll auxiliaryStack, ll destinationStack ){
    if(numberofdisk == 1){
        moves.push_back({sourceStack, destinationStack}); 
        return; 
    }
    moveDisk(numberofdisk - 1, moves, sourceStack, destinationStack, auxiliaryStack); 
    moves.push_back({sourceStack, destinationStack}); 
    moveDisk(numberofdisk - 1,moves, auxiliaryStack,sourceStack,destinationStack); 
}
void towerofhanoi(ll numberofdisk){
    vector<vector<ll> > moves; 
    ll sourceStack = 1; 
    ll auxiliaryStack = 2; 
    ll destinationStack = 3; 
    moveDisk(numberofdisk, moves, sourceStack, auxiliaryStack, destinationStack); 
    cout << moves.size() << endl; 
    for(auto move : moves){ 
        cout << move[0] << " " << move[1] << endl; 
    }
}
int main(){
    ll n; 
    cin >> n; 
    towerofhanoi(n); 
    return 0; 
}