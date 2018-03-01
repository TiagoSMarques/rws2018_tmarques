#include <iostream>

class Player
{
public:
  // Constructor with the same name as the class
  Player(std::string name)
  {
    this->name = name;
  }

  int setTeamName(int team_index = 0 /*default value*/)
  {
    switch (team_index)
    {
      case 0:
        return setTeamName("red");
        break;
      case 1:
        return setTeamName("green");
        break;
      case 2:
        return setTeamName("blue");
        break;
      default:
        std::cout << "wrong team index given. Cannot set team" << std::endl;
        break;
    }
  }

  // Set team name, if given a correct team name (accessor)
  int setTeamName(std::string team)
  {
    if (team == "red" || team == "green" || team == "blue")
    {
      this->team = team;
      return 1;
    }
    else
    {
      this->team = "no team";
      std::cout << "cannot set team name to " << team << std::endl;
      return 0;
    }
  }

  // Gets team name (accessor)
  std::string getTeam(void)
  {
    return team;
  }

  std::string name;  // A public atribute

private:
  std::string team;
};

class MyPlayer : public Player
{
public:
  MyPlayer(std::string argin_name, std::string argin_team) : Player(argin_name)
  {
    setTeamName(argin_team);
  }
};

int main()
{
  // Creating an instance of class Player
  Player player("tmarques");
  player.setTeamName(2);

  // std::cout << "somar = " << somar(.2, 2.7) << std::endl;

  std::cout << "player.name is " << player.name << std::endl;
  std::cout << "team is " << player.getTeam() << std::endl;

  MyPlayer my_player("mtiago", "green");
  std::cout << "team is " << my_player.getTeam() << std::endl;
}