using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using System.Runtime.InteropServices;

namespace ObjectTracker
{
	public unsafe class SecondWindow : GameWindow
	{
		//set these two variables in OnUpdateFrame
		private Quaternion rotation = Quaternion.Identity;
		private Vector3 position;

		//temporary variable to get the box spinning until we get data in
		private float tmpAngle;
		private bool tmpMoveUp;

		private int boxVbo, boxIbo;

		const float gyroScaleDiv = 14.375f;
		const float deg2rad = MathHelper.Pi / 180f;
		const float accelScaleDiv = 256f;
		const float gain = 0.05f;
		const float gravity = 9.806f;

		const float radsPerPixel = MathHelper.PiOver4 / 1024f;
		const float ledDistance = 50.8f; //in mm

		Vector3 ir_position;

		int index;
		Data d;
		Vector3 gyroOff;
		Vector3 accelOff;
		Vector3 velocity;
		Vector3 gravityVec = new Vector3(0, 0, gravity);
		Vector3 prevPosition = new Vector3(0, 0, 0);
		Quaternion Qinv;
		Vector3 up;

		FloatFilter tiltSensorFilter = new FloatFilter(1000);
		VectorFilter pastAccels = new VectorFilter(1000);

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

			RawHidDevice.rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
		}


		protected override void OnUpdateFrame(FrameEventArgs e)
		{
			base.OnUpdateFrame(e);

			int result = 0;
			fixed (Data* dp = &d)
			{
				result = RawHidDevice.rawhid_recv(0, dp, 64, 2);
			}

			if (d.check != 1337)
				Console.WriteLine("ERROR" + d.check + " " + sizeof(Data));

			rotation = d.q;
			//position = d.a;
			Console.WriteLine(d.q);
			Console.WriteLine(d.a);
			Console.WriteLine(d.m);
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

		protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
		{
			base.OnClosing(e);

			RawHidDevice.rawhid_close(0);
		}

		[StructLayout(LayoutKind.Sequential, Pack = 1)]
		public struct Data
		{
			public Quaternion q;
			public Vector3 a;
			public Vector3 m;
			public short f1, f2, f3, f4;
			public float p0, p1, p2;
			public int check;
		}
	}
}
