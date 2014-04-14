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
	public unsafe class MainWindow : GameWindow
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
        Vector3 up2;

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
			int result = 0;
			gyroOff=Vector3.Zero;
			accelOff=Vector3.Zero;
			fixed (Data* dp = &d)
			{
				for (int i = 0; i < 100; i++)
				{
					result = RawHidDevice.rawhid_recv(0, dp, 64, 1);

					//accelOff += new Vector3(d.ax / accelScaleDiv * gravity, d.ay / accelScaleDiv * gravity, d.az / accelScaleDiv * gravity);
					gyroOff.X += dp->rx / gyroScaleDiv * deg2rad;
					gyroOff.Y += dp->ry / gyroScaleDiv * deg2rad;
					gyroOff.Z += dp->rz / gyroScaleDiv * deg2rad;
				}
				
				
				gyroOff.X = gyroOff.X / 100f;
				gyroOff.Y = gyroOff.Y / 100f;
				gyroOff.Z = gyroOff.Z / 100f;
				accelOff = new Vector3(d.ax / accelScaleDiv * gravity, d.az / accelScaleDiv * gravity, d.ay / accelScaleDiv * gravity);
				accelOff -= Vector3.Normalize(accelOff) * 9.806f;
				printData("accel off", accelOff);
			}
		}


		protected override void OnUpdateFrame(FrameEventArgs e)
		{
			base.OnUpdateFrame(e);

			int result = 0;
			fixed (Data* dp = &d)
			{
				result = RawHidDevice.rawhid_recv(0, dp, 64, 2);
				Vector3 tmp = new Vector3(d.rx, d.ry, d.rz);
			}

			if (d.packetCount != 1337)
				Console.WriteLine("ERROR" + d.packetCount + " " + sizeof(Data));

			//accel data
			Vector3 accelData;

			accelData.X = (d.ax / accelScaleDiv) * gravity;
			accelData.Y = (d.az / accelScaleDiv) * gravity;
			accelData.Z = (d.ay / accelScaleDiv) * gravity;
			//accelData -= accelOff;
			//accelData -= gravityVec;

			Qinv = Quaternion.Invert(rotation);
			Vector3 up = Vector4.Transform(new Vector4(0, 1f, 0, 0), Qinv).Xyz;

			//gyro data
			Vector3 gyroData;
			gyroData.X = d.rx / gyroScaleDiv * deg2rad;
			gyroData.Y = d.ry / gyroScaleDiv * deg2rad;
			gyroData.Z = d.rz / gyroScaleDiv * deg2rad;
			gyroData -= gyroOff;
            
			Qinv = Quaternion.Invert(rotation);
			Vector3 down = Vector3.Transform(new Vector3(0, 1f, 0), Qinv);
			down *= gravity;
			accelData -= down;
			//mag data
			Vector3 mag;
			mag.X = d.mx * deg2rad;
			mag.Y = d.my * deg2rad;
			mag.Z = d.mz * deg2rad;

			//printData("accel", accelData);
			//printData("mag", mag);
			float heading = (float)Math.Atan2(mag.Y, mag.X);
			if (heading < 0)
				heading += MathHelper.TwoPi;
			Console.WriteLine(heading * 180f / MathHelper.Pi);
            //Console.Out.WriteLine(heading);

			const float spikeThreshold = 0.01f;
			const float gravityThreshold = 0.1f;
			float proportionalGain = 5 * gain;
			float integralGain = 0.0125f;
			
			Vector3 tiltCorrect = CalculateTiltCorrection(accelData, up);

			float tiltAngle = Vector3.CalculateAngle(up, accelData);
			tiltSensorFilter.Add(tiltAngle);
			if (tiltAngle > tiltSensorFilter.Mean + spikeThreshold)
				proportionalGain = integralGain = 0;

			if (Math.Abs(accelData.Length / gravity - 1) > gravityThreshold)
				integralGain = 0;

			gyroData += (tiltCorrect * proportionalGain);
			gyroOff -= (tiltCorrect * integralGain * (float)e.Time);

			if (Math.Abs(accelData.Z) >= 9.7 && Math.Abs(accelData.Z) <= 9.9)
			{
				// printData("tiltCorrect", tiltCorrect);
			}

			/*
			velocity += accelData * (float)e.Time;
			Vector3 displacement = velocity * (float)e.Time;
		    position += displacement;
		   printData("velocity",velocity);
			printData("accel", accelData);
			printData("displace", displacement);*/
			//mag data
			float ang = gyroData.Length * ((float)e.Time);
			Vector3 axis = Vector3.Normalize(gyroData);

			if (ang > 0)
				rotation *= Quaternion.FromAxisAngle(axis, ang);

			if (index % 500 == 0)
				rotation = Quaternion.Normalize(rotation);

			//rotation = Quaternion.FromAxisAngle(Vector3.UnitY, heading);

			

			//velocity += accelData * (float)e.Time;
			//position += velocity * (float)e.Time;


			//print function for debugging, prints label and x y z of vector
			//printData("gyro", gyroData);
		  //  printData("accel", accelData);
		  //  Console.WriteLine(accelData.Length);
		  //  printData("mag", magData);
            //  printData("gyro off", gyroOff);

            Vector3 gravityCorrect = gravity * Vector3.Normalize(rotation.ToAxisAngle().Xyz);
           // accelData += gravityCorrect;

            //printData("accel", accelData);
          // position = (0.5f * accelData + 1.5f * pastAccels.Prev() + 2 * (pastAccels.Sum - pastAccels.Prev())) * 0.5f * (float)e.Time * (float)e.Time + prevPosition;
        //   position *= 0.7f;
            pastAccels.Add(accelData);
            prevPosition = position;
            //printData("position", position);
            //printData("accel", accelData);
	
            index++;
		}

		Vector3 CalculateTiltCorrection(Vector3 current, Vector3 estimate)
		{
			current = Vector3.Normalize(current);
			estimate = Vector3.Normalize(estimate);
			Vector3 corrected = Vector3.Cross(current,estimate);
			float cosError = Vector3.Dot(current,estimate);
			return corrected * (float)Math.Sqrt(2 / (1 + cosError + float.Epsilon));
		}

		void printData(string t, Vector3 d)
		{
			Console.WriteLine(t + " x = " + d.X);
			Console.WriteLine(t + " y = " + d.Y);
			Console.WriteLine(t + " z = " + d.Z);
			Console.WriteLine("");
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
			public byte ab;
			public byte cd;
			public short ax;
			public short ay;
			public short az;
			public short mx;
			public short my;
			public short mz;
			public short rx;
			public short ry;
			public short rz;
			public long p0, p1, p2, p3, p4;
			public short p5;
			public short packetCount;

			public string ToCsv()
			{
				return ax + ", " + ay + ", " + az + ", " + mx + ", " + my + ", " + mz + ", " + rx + ", " + ry + ", " + rz + ", " + packetCount;
			}

			public override string ToString()
			{
				return "#" + packetCount + " " + ab.ToString("X2") + ":" + cd.ToString("X2") + " : (" + ax + ", " + ay + ", " + az + ") : (" + mx + ", " + my + ", " + mz + ") : (" + rx + ", " + ry + ", " + rz + ")";
			}
		}
	}
}
