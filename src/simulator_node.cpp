#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>

#include <futrobotros/TeamPose.h>
#include <futrobotros/TeamPWM.h>

#include "data.h"
#include "modelo.h"
#include "system.h"
#include "functions.h"
// #include <stdio.h>
// #include <cstring>

#define T_AMOSTR 0.033

ros::Publisher yellow_team_poses_pub;
ros::Publisher blue_team_poses_pub;
ros::Publisher ball_pub;

static Modelo est; 
static GAME_STATE estado_simulacao = PLAY_STATE;

// atualiza sinal de controle
void atualiza_controle(PWM_ROBOTS &controle, TEAM cor, Modelo &est)
{
  for (int id=0; id<3; id++) {
    // Esquerdo
    est.atualizar_controle(cor, id, MOTOR_ESQUERDO, controle.me[id].left);
    // Direito
    est.atualizar_controle(cor, id, MOTOR_DIREITO, controle.me[id].right);
  }
}

void yellowTeamControlCallback(const futrobotros::TeamPWM::ConstPtr& msg)
{
	SINAL_RADIO meuCtrl;
	// meuCtrl.id = ?;
	for (int i = 0; i < 3; i++) {
		meuCtrl.c.me[i].left = msg->robot_pwm[i].left;
		meuCtrl.c.me[i].right = msg->robot_pwm[i].right;
	}
    atualiza_controle(meuCtrl.c, YELLOW_TEAM, est);
}

void blueTeamControlCallback(const futrobotros::TeamPWM::ConstPtr& msg)
{
	SINAL_RADIO meuCtrl;
	// meuCtrl.id = ?;
	for (int i = 0; i < 3; i++) {
		meuCtrl.c.me[i].left = msg->robot_pwm[i].left;
		meuCtrl.c.me[i].right = msg->robot_pwm[i].right;
	}
    atualiza_controle(meuCtrl.c, BLUE_TEAM, est);
}

void simulate()
{
	SITUACAO minhaSit;
	double t, dt, dsusp, damostr = 0.0;

	est.posicao_inicial();
	minhaSit.id = 0;      // Vai gerar a primeira imagem
	damostr = 0.0;

	/* ***************************************
	   laço (enquanto simulador estiver ativo)
	   *************************************** */
	ros::Rate r(100); // 100 hz
	while (ros::ok() && estado_simulacao != FINISH_STATE) {
		// Leitura de eventuais sinais de controle
		ros::spinOnce();

		// Lê a hora atual
		t = relogio();

		// Intervalo desde o último ciclo
		dt = t - est.le_tempo();
		if (estado_simulacao == PLAY_STATE) {
			est.simular(dt);
		}
		else {  // SUSPESO
			est.avancar_sem_simular(dt);
			dsusp += dt;
		}
		damostr += dt;
		if (estado_simulacao == PAUSE_STATE) {
			dsusp += dt;
		}
		else {
			dsusp = 0.0;
		}

		// Envio de imagem a cada período de amostragem
		if ( damostr >= T_AMOSTR ) {
			est.le_posicao(minhaSit.pos);
			//normalização dos ângulos
			for (int i = 0; i < 3; i++) {
				minhaSit.pos.azul[i].theta() = ang_equiv(minhaSit.pos.azul[i].theta());
				minhaSit.pos.amrl[i].theta() = ang_equiv(minhaSit.pos.amrl[i].theta());
			}
			
			ros::Time t = ros::Time::now();

			futrobotros::TeamPose msg;
			for (int i = 0; i < 3; i++) {
				msg.robot_pose[i].x = minhaSit.pos.azul[i].x();
				msg.robot_pose[i].y = minhaSit.pos.azul[i].y();
				msg.robot_pose[i].theta = minhaSit.pos.azul[i].theta();
			}
			blue_team_poses_pub.publish(msg);

			for (int i = 0; i < 3; i++) {
				msg.robot_pose[i].x = minhaSit.pos.amrl[i].x();
				msg.robot_pose[i].y = minhaSit.pos.amrl[i].y();
				msg.robot_pose[i].theta = minhaSit.pos.amrl[i].theta();
			}
			yellow_team_poses_pub.publish(msg);

			geometry_msgs::PointStamped ball_msg;
			ball_msg.header.frame_id = "origin";
			ball_msg.header.stamp = t;
			// ball_msg.header.seq = ??;
			ball_msg.point.x = minhaSit.pos.bola.x();
			ball_msg.point.y = minhaSit.pos.bola.y();
			ball_pub.publish(ball_msg);

			damostr -= T_AMOSTR;
			minhaSit.id++;  // id do quadrado gerado
		} // fim if ( damostr >= T_AMOSTR )

	// 	// Verifica se houve gols marcados
	// 	/*
	// 	if (estado_simulacao == PLAY_STATE &&
	// 	fabs(minhaSit.pos.bola.x()) > FIELD_WIDTH/2.0) {
	// 	  if (minhaSit.pos.bola.x() > 0.0) {
	// 	minhaSit.gols.azul++;
	// 	  }
	// 	  else {
	// 	minhaSit.gols.amrl++;
	// 	  }
	// 	  estado_simulacao = minhaSit.estado = PAUSE_STATE;
	// 	  est.posicao_inicial();
	// 	}
	// 	*/
	// 	// Verifica se já deve sair do estado PAUSE_STATE
	// 	if (estado_simulacao == PAUSE_STATE && dsusp > 10) {
	// 		estado_simulacao = PLAY_STATE;
	// 		dsusp = 0.0;
	// 	}

		r.sleep(); // 10ms = tick do Linux
	} //fim while (estado_simulacao != FINISH_STATE)

}


int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "simulator");
	
	// Init a node handler
	ros::NodeHandle n;

	// Get namespace parameters
	std::string yellow_ns, blue_ns;
	n.param<std::string>("yellow_ns", yellow_ns, "yellow");
	n.param<std::string>("blue_ns", blue_ns, "blue");

	// Set up topics to publish simulated data
	yellow_team_poses_pub = n.advertise<futrobotros::TeamPose>(yellow_ns + "/team_poses", 1000);
	blue_team_poses_pub = n.advertise<futrobotros::TeamPose>(blue_ns + "/team_poses", 1000);
	ball_pub = n.advertise<geometry_msgs::PointStamped>("ball_position", 1000);

	// Subscribe to the topic with the acquired images
	ros::Subscriber sub_yellow_team_control = n.subscribe(yellow_ns + "/team_pwms", 1000, yellowTeamControlCallback);
	ros::Subscriber sub_blue_team_control = n.subscribe(blue_ns + "/team_pwms", 1000, blueTeamControlCallback);

	// Start the simulation, returns only when finished.
	simulate();

	return 0;
}
