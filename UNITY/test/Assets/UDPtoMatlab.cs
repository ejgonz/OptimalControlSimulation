/*
-----------------------
UDP Object
-----------------------
Code pulled from here and modified
 [url]http://msdn.microsoft.com/de-de/library/bb979228.aspx#ID0E3BAC[/url]
Unity3D to MATLAB UDP communication 
Modified by: Sandra Fang 
2016
*/

using UnityEngine;
using System.Collections;

using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

public class UDPtoMatlab : MonoBehaviour
{
	#region msg commands
	// Message headers (to Matlab)
	private const float INITIALIZE_CMD = 120;
	private const int DATA_CMD = 121;
	private const int END_CMD = 122;
	// Message headers (from Matlab)
	private const int READY_CMD = 220;
	private const int CONTROL_CMD = 221;
	#endregion msg commands

	#region setup parameters
	// Parameters to send to Matlab for creating optimizer
	public const float T 	 = 6;		  // Horizon
	public const float x1max = 5.0f;      // Environment limits (cm)
	public const float x1min = -55.0f;
	public const float x2max = 7.5f;
	public const float x2min = -20.0f;
	public const float maxSpeed = 60.0f;  	// robot speed limit (cm/s)
	private const float r = 0.0f;	      	// control penalty
	public const float safetyDist = 6.0f;  // collision avoidance
	private const float numTargets = 3;
		// Target positions
		private const float xtarget1_x = 1.4f; private const float xtarget1_y = -14.3f;
		private const float xtarget2_x = -28.6f; private const float xtarget2_y = 5.1f;
		private const float xtarget3_x = -53.0f; private const float xtarget3_y = -14.3f;
	#endregion setup parameters

	#region measured variables
	private Vector2 robotPosLeft  = new Vector2(-35.0f, -10.0f);	// robot positions
	private Vector2 robotPosRight = new Vector2(-20.0f, 0.0f);  //
	private float numPrevPoints = 3;
	private Vector2 lastHandPosLeft  	 = new Vector2(-27.25f, -22.859f);		 // Left Hand history
	private Vector2 lastLastHandPosLeft  = new Vector2(-30.287f, -24.90f);  	 // 
	private Vector2 lastLastLastHandPosLeft  = new Vector2(-27.25f, -26.690f);   // 
	private Vector2 lastHandPosRight  	 = new Vector2(-24.82614f, -22.1586f);       // Right Hand history
	private Vector2 lastLastHandPosRight  = new Vector2(-24.44288f, -24.6778f);      // 
	private Vector2 lastLastLastHandPosRight  = new Vector2(-24.2559f, -26.0019f);   // 
	#endregion measured variables

	#region variables
	private bool initialized = false;
	private Vector2 commandVelLeft  = new Vector2(0.0f, 0.0f);	// left robot control (from MATLAB)
	private Vector2 commandVelRight = new Vector2(0.0f, 0.0f);  // right robot control (from MATLAB)
	private float startTime = 0.0f;
    public float dataRateHz = 0.5f;
	private float dt;  // timestep (s)
	private Stopwatch stopWatch = new Stopwatch();
	private bool responseRecievied = true;
	#endregion variables

	//local host
	public string IP = "127.0.0.1";

	//Ports
	public int portLocal = 8000;
	public int portRemote = 8001;

	// Create necessary UdpClient objects
	UdpClient client;
	IPEndPoint remoteEndPoint;

	// Receiving Thread
	Thread receiveThread;

	// start from Unity3d
	public void Start ()
	{
		stopWatch.Start ();
		dt = 1 / dataRateHz;
		initUDP ();
	}
		

	// Initialization code
	private void initUDP ()
	{
		// Initialize (seen in comments window)
		print ("UDP Object init()");

		// Create remote endpoint (to Matlab) 
		remoteEndPoint = new IPEndPoint (IPAddress.Parse (IP), portRemote);

		// Create local client
		client = new UdpClient (portLocal);

		// local endpoint define (where messages are received)
		// Create a new thread for reception of incoming messages
		receiveThread = new Thread (
			new ThreadStart (CommManager));
		receiveThread.IsBackground = true;
		receiveThread.Start ();

		// Send MATLAB initialize command
		float[] setupMsg = {
			INITIALIZE_CMD,
			T,
			x1max, x1min, x2max, x2min,
			maxSpeed,
			r,
			dt,
			safetyDist,
			numTargets,
			xtarget1_x, xtarget1_y,
			xtarget2_x, xtarget2_y,
			xtarget3_x, xtarget3_y
		};
		sendData (setupMsg);
		Debug.Log ("Sent Initialization Request");
	}

	/// <summary>
	/// Manages communication between Unity and Matlab
	/// </summary>
	private void CommManager ()
	{
		while (true) {
			// Check for new messages
			var msg = recieveData ();
			
			// if new message recieved
			if (msg[0] > 0) {
				// Read header
				int msgHeader = (int)msg[0];
			
				// Take appropriate action
				switch (msgHeader) {

					case READY_CMD: 
						Debug.Log ("MATLAB script initialized.");
						initialized = true;
						startTime = (float)stopWatch.Elapsed.TotalSeconds;
						break;
					case CONTROL_CMD:
						responseRecievied = true;
						// Parse control data
						commandVelRight.x = msg [1];
						commandVelRight.y = msg [2];
						commandVelLeft.x = msg [3];
						commandVelLeft.y = msg [4];
						Debug.Log (stopWatch.Elapsed.TotalSeconds - startTime);
						Debug.Log ("Recieved control packet.");
						break;
					default:
						Debug.Log ("Unexpected message header");
						break;
				}
			}

			// Send data at specified rate
			if (responseRecievied) { //stopWatch.Elapsed.TotalSeconds - startTime > dt
				if (initialized && responseRecievied) {
					// prepare and send standard data packet
					// Send MATLAB initialize command
					float[] dataMsg = {
						DATA_CMD,
						robotPosLeft.x, robotPosLeft.y,
						robotPosRight.x, robotPosRight.y,
						numPrevPoints,
						lastLastLastHandPosLeft.x, lastLastLastHandPosLeft.y,
						lastLastHandPosLeft.x, lastLastHandPosLeft.y,
						lastHandPosLeft.x, lastHandPosLeft.y,
						lastLastLastHandPosRight.x, lastLastLastHandPosRight.y,
						lastLastHandPosRight.x, lastLastHandPosRight.y,
						lastHandPosRight.x, lastHandPosRight.y
					};
					sendData (dataMsg);
					Debug.Log ("Sent Data");

					// reset timer
					startTime = (float)stopWatch.Elapsed.TotalSeconds;
					responseRecievied = false;
				}
			}



		}
	}


	// Recieve Data
	private float[] recieveData () {
		try {
			IPEndPoint anyIP = new IPEndPoint (IPAddress.Any, 0);

			if (client.Available > 0) {
				// Read message
				byte[] dataBytes = client.Receive (ref anyIP);
				return GetFloats(dataBytes);
			} else {
				// No new message
				return new float[] {-2};
			}

		} catch (Exception err) {
			print (err.ToString ());
			return new float[] {-1};
		}
	}

	// Send data
	private void sendData (float[] msg)
	{
		try {
			byte[] data = GetBytes(msg);
			client.Send (data, data.Length, remoteEndPoint);
		} catch (Exception err) {
			print (err.ToString ());
		}
	}

	// Convert float array to bytes (for sending data)
	static byte[] GetBytes(float[] values)
	{
		var result = new byte[values.Length * sizeof(float)];
		Buffer.BlockCopy(values, 0, result, 0, result.Length);
		return result;
	}

	// Convert bytes to float array (for recieving data)
	static float[] GetFloats(byte[] bytes)
	{
		var result = new float[bytes.Length / sizeof(float)];
		Buffer.BlockCopy(bytes, 0, result, 0, bytes.Length);
		return result;
	}

	//Prevent crashes - close clients and threads properly!
	void OnDisable ()
	{ 
		// Send exit command to MATLAB
		float[] msg = {END_CMD};
		sendData (msg);

		if (receiveThread != null)
			receiveThread.Abort (); 

		client.Close ();
	}

	public void PrintByteArray(byte[] bytes)
	{
		var sb = new StringBuilder("new byte[] { ");
		foreach (var b in bytes)
		{
			sb.Append(b + ", ");
		}
		sb.Append("}");
		Debug.Log(sb.ToString());
	}

}