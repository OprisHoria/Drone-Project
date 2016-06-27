using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;

namespace WindowsFormsApplication2
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            serialPort1.PortName = "COM3";
            serialPort1.BaudRate = 9600;
            //serialPort1.Open();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (!serialPort1.IsOpen)
            {
                try
                {
                    serialPort1.Open();
                    serialPort1.Write("T");
                    serialPort1.Close();
                }
                catch
                {
                    MessageBox.Show("There was an error. Please make sure that the correct port was selected, and the device, plugged in.");
                }
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {/*
            if (!serialPort1.IsOpen)
            {
                try
                {
                    serialPort1.Open();
                    serialPort1.Write("0");
                    serialPort1.Close();
                }
                catch
                {
                    MessageBox.Show("There was an error. Please make sure that the correct port was selected, and the device, plugged in.");
                }
            }*/
            serialPort1.Write("0");
        }

        private void button3_Click(object sender, EventArgs e)
        {
            serialPort1.Write("52");
            Console.WriteLine("Upped all");
        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            serialPort1.Write("0");

            string returnMessage = "";
            returnMessage = serialPort1.ReadExisting();

            richTextBox1.Text = returnMessage;
            Console.WriteLine("Requested temperature display");
        }

        private void button4_Click(object sender, EventArgs e)
        {
            serialPort1.Write("7");

            string returnMessage = "";
            returnMessage = serialPort1.ReadExisting();

            richTextBox1.Text = returnMessage;
            Console.WriteLine("Requested take off");
        }

        private void button5_Click(object sender, EventArgs e)  // FRONT LEFT
        {
            serialPort1.Write("4");

            string returnMessage = "";
            returnMessage = serialPort1.ReadExisting();

            richTextBox1.Text = returnMessage;
            Console.WriteLine("Requested leaning to the front left");
        }

        private void button6_Click(object sender, EventArgs e)  // FRONT RIGHT
        {
            serialPort1.Write("5");

            string returnMessage = "";
            returnMessage = serialPort1.ReadExisting();

            richTextBox1.Text = returnMessage;
            Console.WriteLine("Requested leaning to the front right");
        }

        private void button7_Click(object sender, EventArgs e)  // REAR LEFT
        {
            serialPort1.Write("6");

            string returnMessage = "";
            returnMessage = serialPort1.ReadExisting();

            richTextBox1.Text = returnMessage;
            Console.WriteLine("Requested leaning to the rear left");
        }

        private void button8_Click(object sender, EventArgs e)  // REAR RIGHT
        {
            serialPort1.Write("2");

            string returnMessage = "";
            returnMessage = serialPort1.ReadExisting();

            richTextBox1.Text = returnMessage;
            Console.WriteLine("Requested leaning to the rear right");
        }

        private void button9_Click(object sender, EventArgs e)  // STOP
        {
            serialPort1.Write("9");
            Console.WriteLine("STOP");

            string returnMessage = "";
            returnMessage = serialPort1.ReadExisting();

            richTextBox2.Text = returnMessage;
        }

        private void button3_Click_1(object sender, EventArgs e)    //Decrease speed
        {
            serialPort1.Write("0");
            Console.WriteLine("Decrease speed");

            string returnMessage = "";
            returnMessage = serialPort1.ReadExisting();

            richTextBox2.Text = returnMessage;
        }

        private void button2_Click_1(object sender, EventArgs e)    //Increase speed
        {
            string returnMessage = "";

            serialPort1.Write("1");
            Console.WriteLine("Increased speed");

            returnMessage = serialPort1.ReadExisting();
            richTextBox2.Text = returnMessage;
        }
        
        private void richTextBox2_TextChanged(object sender, EventArgs e)
        {
            
        }

        private void richTextBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void button10_Click(object sender, EventArgs e)
        {
            serialPort1.Write("8");

            string returnMessage = "";
            returnMessage = serialPort1.ReadExisting();

            richTextBox1.Text = returnMessage;
            Console.WriteLine("Requested Landing");
        }
    }
}
