using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;

namespace ObjectTracker
{
	public class MainWindow : GameWindow
	{
		//set these two variables in OnUpdateFrame
		private Quaternion rotation = Quaternion.Identity;
		private Vector3 position;

		//temporary variable to get the box spinning until we get data in
		private float tmpAngle;
		private bool tmpMoveUp;

		private int boxVbo, boxIbo;

		const float dt = 0.001f;

		List<Vector3> acceleration = new List<Vector3>();
		List<Vector3> gyro = new List<Vector3>();
		int index;

		private static readonly byte[] boxInds =
		{
			0,  2,  1,  0,  3,  2,  //-Z
			4,  6,  5,  4,  7,  6,  //+X
			8,  10, 9,  8,  11, 10, //+Z
			12, 14, 13, 12, 15, 14, //-X
			16, 18, 17, 16, 19, 18, //+Y
			20, 22, 21, 20, 23, 22  //-Y
		};

		private static readonly float[] boxVerts =
		{
			//-Z face
			-0.5f,  0.5f, -0.5f,  0,  0, -1,
			-0.5f, -0.5f, -0.5f,  0,  0, -1,
			 0.5f, -0.5f, -0.5f,  0,  0, -1,
			 0.5f,  0.5f, -0.5f,  0,  0, -1,

			//+X face
			 0.5f,  0.5f, -0.5f,  1,  0,  0,
			 0.5f, -0.5f, -0.5f,  1,  0,  0,
			 0.5f, -0.5f,  0.5f,  1,  0,  0,
			 0.5f,  0.5f,  0.5f,  1,  0,  0,

			//+Z face
			 0.5f,  0.5f,  0.5f,  0,  0,  1,
			 0.5f, -0.5f,  0.5f,  0,  0,  1,
			-0.5f, -0.5f,  0.5f,  0,  0,  1,
			-0.5f,  0.5f,  0.5f,  0,  0,  1,

			//-X face
			-0.5f,  0.5f,  0.5f, -1,  0,  0,
			-0.5f, -0.5f,  0.5f, -1,  0,  0,
			-0.5f, -0.5f, -0.5f, -1,  0,  0,
			-0.5f,  0.5f, -0.5f, -1,  0,  0,

			//+Y face
			-0.5f,  0.5f,  0.5f,  0,  1,  0,
			-0.5f,  0.5f, -0.5f,  0,  1,  0,
			 0.5f,  0.5f, -0.5f,  0,  1,  0,
			 0.5f,  0.5f,  0.5f,  0,  1,  0,

			//-Y face
			-0.5f, -0.5f, -0.5f,  0, -1,  0,
			-0.5f, -0.5f,  0.5f,  0, -1,  0,
			 0.5f, -0.5f,  0.5f,  0, -1,  0,
			 0.5f, -0.5f, -0.5f,  0, -1,  0
		};

		protected override void OnLoad(EventArgs e)
		{
			base.OnLoad(e);

			boxVbo = GL.GenBuffer();
			GL.BindBuffer(BufferTarget.ArrayBuffer, boxVbo);
			GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr)(boxVerts.Length * 4), boxVerts, BufferUsageHint.StaticDraw);
			GL.BindBuffer(BufferTarget.ArrayBuffer, 0);

			boxIbo = GL.GenBuffer();
			GL.BindBuffer(BufferTarget.ElementArrayBuffer, boxIbo);
			GL.BufferData(BufferTarget.ElementArrayBuffer, (IntPtr)boxInds.Length, boxInds, BufferUsageHint.StaticDraw);
			GL.BindBuffer(BufferTarget.ElementArrayBuffer, 0);

			GL.Enable(EnableCap.DepthTest);
			GL.DepthMask(true);
			GL.DepthFunc(DepthFunction.Lequal);
			GL.Enable(EnableCap.CullFace);
			GL.FrontFace(FrontFaceDirection.Ccw);
			GL.Enable(EnableCap.Blend);
			GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);
			GL.ClearColor(Color4.CornflowerBlue);

			GL.Enable(EnableCap.Lighting);
			GL.Enable(EnableCap.Light0);
			GL.Light(LightName.Light0, LightParameter.Ambient, new Vector4(0.6f, 0.6f, 0.6f, 1f));

			string[] lines = File.ReadAllLines("imu_data.csv");
			string[][] data = new string[lines.Length-1][];
			string[] s1 = lines[0].Split(',');
			for(int i=0;i<lines.Length-1;i++)
			{
				string[] s = lines[i+1].Split(',');
				data[i] = s;
			}

			const float deg2rad = MathHelper.Pi / 180f;

			for (int i = 0; i < data.Length;i++)
			{
				Vector3 tmp;
				tmp.X = float.Parse(data[i][0]) / 32768f;
				tmp.Y = float.Parse(data[i][1]) / 32768f;
				tmp.Z = float.Parse(data[i][2]) / 32768f;
				acceleration.Add(tmp);
				tmp.X = float.Parse(data[i][6]) / 16.384f * deg2rad;
				tmp.Y = float.Parse(data[i][7]) / 16.384f * deg2rad;
				tmp.Z = float.Parse(data[i][8]) / 16.384f * deg2rad;
				gyro.Add(tmp);
			}

			Console.WriteLine(acceleration.Min(v => v.X));
			Console.WriteLine(acceleration.Max(v => v.X));
			Console.WriteLine(gyro.Min(v => v.X));
			Console.WriteLine(gyro.Max(v => v.X));

		}

		protected override void OnUpdateFrame(FrameEventArgs e)
		{
			base.OnUpdateFrame(e);

			Vector3 gyroData = gyro[index];
			float ang = gyroData.Length * dt;
			Vector3 axis = Vector3.Normalize(gyroData);

			rotation *= Quaternion.FromAxisAngle(axis, ang);


			index++;
			//temporary motion. replace this
			/*tmpAngle += MathHelper.Pi / 30f;
			tmpAngle %= MathHelper.TwoPi;

			rotation = Quaternion.FromAxisAngle(new Vector3(0, 1, 1), tmpAngle);


			if (tmpMoveUp)
				position.Y += 0.1f;
			else
				position.Y -= 0.1f;

			if (position.Y >= 2f)
				tmpMoveUp = false;
			else if (position.Y <= -2f)
				tmpMoveUp = true;*/
		}

		protected override void OnRenderFrame(FrameEventArgs e)
		{
			GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

			GL.EnableClientState(ArrayCap.VertexArray);
			GL.EnableClientState(ArrayCap.NormalArray);

			GL.Enable(EnableCap.Lighting);
			GL.Enable(EnableCap.Light0);
			GL.Light(LightName.Light0, LightParameter.Position, new Vector4(0f, 1, 0f, 0));

			GL.BindBuffer(BufferTarget.ArrayBuffer, boxVbo);
			GL.VertexPointer(3, VertexPointerType.Float, 6 * 4, 0);
			GL.NormalPointer(NormalPointerType.Float, 6 * 4, 3 * 4);

			GL.BindBuffer(BufferTarget.ElementArrayBuffer, boxIbo);

			//camera offset
			GL.LoadIdentity();
			GL.Translate(0, 0, -5);

			Matrix4 transf = Matrix4.CreateFromQuaternion(rotation);
			transf *= Matrix4.CreateTranslation(position);
			GL.MultMatrix(ref transf);
			GL.DrawElements(PrimitiveType.Triangles, boxInds.Length, DrawElementsType.UnsignedByte, 0);

			GL.BindBuffer(BufferTarget.ElementArrayBuffer, 0);
			GL.BindBuffer(BufferTarget.ArrayBuffer, 0);

			GL.DisableClientState(ArrayCap.NormalArray);
			GL.DisableClientState(ArrayCap.VertexArray);

			SwapBuffers();
		}

		protected override void OnResize(EventArgs e)
		{
			base.OnResize(e);

			GL.Viewport(0, 0, Width, Height);
			float aspect = Width / (float)Height;

			Matrix4 persp = Matrix4.CreatePerspectiveFieldOfView(MathHelper.PiOver4, aspect, 0.1f, 100f);
			GL.MatrixMode(MatrixMode.Projection);
			GL.LoadMatrix(ref persp);
			GL.MatrixMode(MatrixMode.Modelview);
			GL.LoadIdentity();
		}
	}
}
