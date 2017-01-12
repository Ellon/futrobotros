#include <math.h>
#include <sys/time.h>

#include "futdata.h"

using namespace std;

namespace {
void parse_team(const ros::NodeHandle& n, const string& my_team_str, TEAM& my_team)
{
  if(my_team_str == "BLUE_TEAM")
    my_team = BLUE_TEAM;
  else if(my_team_str == "YELLOW_TEAM")
    my_team = YELLOW_TEAM; 
  else
    throw std::logic_error(string("Value \"") + my_team_str + string("\" is invalid for parameter ") + n.resolveName("team"));
};

void parse_side(const ros::NodeHandle& n, const string& my_side_str, SIDE& my_side)
{
  if(my_side_str == "RIGHT_SIDE")
    my_side = RIGHT_SIDE;
  else if(my_side_str == "LEFT_SIDE")
    my_side = LEFT_SIDE; 
  else
    throw std::logic_error(string("Value \"") + my_side_str + string("\" is invalid for parameter ") + n.resolveName("side"));
}

void parse_game_state(const ros::NodeHandle& n, const string& my_game_state_str, GAME_STATE& game_state)
{
  if(my_game_state_str == "FINISH_STATE")
    game_state = FINISH_STATE;
  else if(my_game_state_str == "PAUSE_STATE")
    game_state = PAUSE_STATE; 
  else if(my_game_state_str == "PENALTY_STATE")
    game_state = PENALTY_STATE;
  else if(my_game_state_str == "FREEKICK_STATE")
    game_state = FREEKICK_STATE;
  else if(my_game_state_str == "GOALKICK_STATE")
    game_state = GOALKICK_STATE;
  else if(my_game_state_str == "FREEBALL_STATE")
    game_state = FREEBALL_STATE;
  else if(my_game_state_str == "INICIALPOSITION_STATE")
    game_state = INICIALPOSITION_STATE;
  else if(my_game_state_str == "PLAY_STATE")
    game_state = PLAY_STATE;
  // else if(my_game_state_str == "CALIBRATION_CONTROL_STATE")
  //   game_state = CALIBRATION_CONTROL_STATE;
  // else if(my_game_state_str == "CALIBRATION_IMAGE_STATE")
  //   game_state = CALIBRATION_IMAGE_STATE;
  else
    throw std::logic_error(string("Value \"") + my_game_state_str + string("\" is invalid for parameter ") + n.resolveName("/game_state"));
}

} // namespace

/** \brief Construtor padrão.
 * \param team A cor do seu time
 * \param side O lado que seu time vai jogar
 * \param mode O modo de jogo.
 */
FutData::FutData(ros::NodeHandle& n)
 : n(n)
{
  string my_team_str;
  if (n.getParam("team", my_team_str))
  {
    parse_team(n, my_team_str, my_team);
  }
  else
  {
    throw std::logic_error(string("Parameter ") + n.resolveName("team") + string(" not set!"));
  }

  string my_side_str;
  if (n.getParam("side", my_side_str))
  {
    parse_side(n, my_side_str, my_side);
  }
  else
  {
    throw std::logic_error(string("Parameter ") + n.resolveName("side") + string(" not set!"));
  }

  // Game state is global
  string game_state_str;
  n.param<string>("/game_state", game_state_str, "PLAY_STATE");
  parse_game_state(n, game_state_str, game_state);

}
