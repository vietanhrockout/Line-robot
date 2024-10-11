#include <iostream> 
#include <vector>
using namespace std; 
#define ll long long 
int main(){
    ll n; 
    cin >> n; 
    ll sum = (n+1)*n/2; 
    if(sum % 2){
        cout << "NO" << endl; 
        return 0; 
    }
    cout << "YES" << endl; 
    vector<ll> set1; 
    vector<ll> set2; 
    ll target = sum/2;
    for(ll i = n; i > 0; i--){
        if(i <= target ){
            set1.push_back(i); 
            target -= i; 
        }else {
            set2.push_back(i); 
        }
    } 
    cout << set1.size() << endl; 
    for(ll num : set1){
        cout << num << " "; 
    }
    cout << "" << endl; 
    cout << set2.size() << endl; 
    for(ll num : set2){
        cout << num << " "; 
    }
    return 0; 
}