/*
  With deepest regrets, I couldn't make it work with ignition::msgs::Int.
  If someone does this, please send a PR.
*/

#include <iostream>
#include <bits/stdc++.h>
#include <string>
#include <ignition/transport.hh>
#include <ignition/msgs.hh>

static const int MAX_BOTS = 10;
// hard-coding this one, fill this value by using system variables. See Swarm Task Round 2017.
static const int BOT_ID = 0;

bool activeBots[MAX_BOTS];

void addBot(const ignition::msgs::StringMsg &_req,ignition::msgs::StringMsg &_rep, bool &_result)
{
  int data = std::stoi(_req.data());
  activeBots[data] = true;
  std::string ans = "";
  for(int i=0;i<MAX_BOTS;i++){
    if(activeBots[i]==true)
      ans+="1";
    else
      ans+="0";
  }
  _rep.set_data(ans);
  _result = true;
}

void removeBot(const ignition::msgs::StringMsg &_req)
{
  int data =std::stoi(_req.data());
  activeBots[data] = false;
}

int main(int argc, char **argv)
{
  // Create a transport node.
  ignition::transport::Node node;
  std::string service_addBot = "/addBot";
  std::string service_removeBot = "/removeBot";

  for(int i=0;i<MAX_BOTS;i++)
    activeBots[i]=false;
  activeBots[BOT_ID]=true;
  
  ignition::msgs::StringMsg req;
  ignition::msgs::StringMsg rep;
  bool result;
  req.set_data(std::to_string(BOT_ID));
  unsigned int timeout = 5000;
  // Should not go for asynchronous. Because, then each bot will reply to its own /addBot request, which might lead to wrong results.
  //node.Request("/addBot", req, responseCb);
  bool executed = node.Request("/addBot",req,timeout, rep, result);
  if(executed){
    if(result){
      std::string temp(rep.data());
      for(int i=0;i<MAX_BOTS;i++){
	if(temp[i] == '1')
	  activeBots[i] = true;
	else
	  activeBots[i] = false;
      }
    }
  }
  
  if (!node.Advertise(service_addBot, addBot))
    {
      std::cerr << "Error advertising service [" << service_addBot << "]" << std::endl;
      return -1;
    }
  
  if (!node.Advertise(service_removeBot, removeBot))
    {
      std::cerr << "Error advertising service [" << service_removeBot << "]" << std::endl;
      return -1;
    }

  // Zzzzzz.
  ignition::transport::waitForShutdown();
  executed = node.Request("/removeBot",req);
}
