using Jitter2;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using JitterDemo.Renderer;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JitterDemo.Demos
{
    public class DemoTestGravity : IDemo
    {
        public string Name => "Demo Test Gravity";

        public void Build()
        {
            Playground pg = (Playground)RenderWindow.Instance;
            World world = pg.World;

            world.SolverIterations = 6;

            pg.ResetScene();

            {
                RigidBody body;
                CylinderShape shape;

/*                body = world.CreateRigidBody();
                body.Position = new Jitter2.LinearMath.JVector(0, 5, -10);
                shape = new CylinderShape(1, 0.5f);
                body.AddShape(shape);*/

/*                body = world.CreateRigidBody();
                body.Position = new Jitter2.LinearMath.JVector(0, 8, -10);
                shape = new CylinderShape(1, 0.5f);
                body.AddShape(shape);*/
            }


            {
                RigidBody body;
                BoxShape shape;

/*                body = world.CreateRigidBody();
                body.Position = new Jitter2.LinearMath.JVector(0, 5, -10);
                shape = new BoxShape(1, 1, 1);
                body.AddShape(shape);

                body = world.CreateRigidBody();
                body.Position = new Jitter2.LinearMath.JVector(0, 8, -10);
                shape = new BoxShape(1, 1, 1);
                body.AddShape(shape);

                body = world.CreateRigidBody();
                body.Position = new Jitter2.LinearMath.JVector(0, 12, -10);
                shape = new BoxShape(1, 1, 1);
                body.AddShape(shape);

                body = world.CreateRigidBody();
                body.Position = new Jitter2.LinearMath.JVector(0, 16, -10);
                shape = new BoxShape(1, 1, 1);
                body.AddShape(shape);*/

                body = world.CreateRigidBody();
                body.Position = new Jitter2.LinearMath.JVector(0, 20, -10);
                shape = new BoxShape(1, 1, 1);
                body.AddShape(shape);

                body = world.CreateRigidBody();
                body.Position = new Jitter2.LinearMath.JVector(0, 24, -10);
                shape = new BoxShape(1, 1, 1);
                body.AddShape(shape);
            }

        }

        public void Draw()
        {
        }

    }
}
