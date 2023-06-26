import datetime
from Window import *
from tkinter import *
import os 
from tkinter.filedialog import askopenfilename
import shutil
import csv

class FrameCSVList(FrameBase):
    """
    A frame which lists the CSVs in the ./CSVs directory onto the screen, including the options to import a new CSV and to use Live Data.
    """

    def __init__(self, master:Tk, main_window, program_window, **args):

         # Retrieve the instance of the simulation frame where the brain data will be displayed.
        self.simulation_frame = program_window.frames["visual_frame"]

        # Set a global variable specifying the frame this subframe is being held in.
        self.program_window = program_window

        # Set a global variable specifying the Window all frames are held in.
        self.main_window = main_window
        
        # Initialise the current frame.
        super().__init__(master, main_window)

        # Disable resizing of the current frame.
        self.pack_propagate(0)  

        # Give the current frame a black border.    
        self.configure(highlightthickness=1, highlightbackground="black")
        
        # Retrieve and display the CSV files to the screen.
        self.display_files()

    def get_file(self, path, name):
        """
        Retrieves the file information for a specified file.
        Retrieves the file name, file extension, modification time, and length.
        """
        
        # Retrieve information about the current file.
        file_stat = os.stat(path)

        # Retrieve the file name from the current file.
        new_file_name = name.split(".")[0]

        # Retrieve the time the file was last modified.
        modification_time = datetime.datetime.fromtimestamp(file_stat.st_mtime).strftime("%d/%m/%y %H:%M:%S")

        # Retrieve the file extension.
        file_extension = os.path.splitext(name)[1]

        # Return the file size.
        file_length = file_stat.st_size

        # Format the information into a dictionary.
        file_info = {
            "name": new_file_name,
            "modification_time": modification_time,
            "extension": file_extension,
            "length": file_length
        }

        # Return the dictionary of information.
        return file_info

    def get_file_info(self, file_name):
        """
        Retrieves the file information for a file.
        """

        # Set the path for the file to be retrieved.
        file_path = os.path.join(self.directory_path, file_name)
        
        # Retrieve the information about the chosen file.
        file_info = self.get_file(file_path, file_name)

        # Append the information about this file to the list of information about every file.
        self.file_info_list.append(file_info)


            

    def display_files(self):
        """
        Display every file in the ./CSVs directory to the screen.
        Display the file name, extension, last modified, and length of the file.
        Display a delete button allowing the user to remove the file from the directory.
        """

        # Retrieve the image to be displayed on the delete button.
        self.delete_image = PhotoImage(file='./Assets/delete.png')

        # Set the path for the CSVs to be retrieved from.
        self.directory_path = "./CSVs"

        # Initialise a list containing the information about each file in the directory.
        self.file_info_list = []

        # For each file in the directory,
        for file_name in os.listdir(self.directory_path):
            # Retrieve information about the selected file.
            self.get_file_info(file_name)

        # set number of rows to number of csv files
        row_length = len(self.file_info_list)

        # set grid propagate to 0 to stop the frame from resizing
        self.grid_propagate(0)

        # set the 1st column to weight 1 to allow the canvas to fill the space
        self.columnconfigure(1,weight=1)
        self.rowconfigure(1, weight=1)

        # Create a canvas with a white background.
        self.canvas = Canvas(self, bg="white")
        self.canvas.grid(row=0, column=0, sticky=NSEW, columnspan=2, rowspan=row_length+1)

        # Create a frame inside the canvas to hold the buttons
        self.frame = Frame(self.canvas, bg="white")

        # Allow the frame to resize.
        self.frame.columnconfigure(0,weight=1)
        

        # Create a scrollbar and associate it with the canvas
        scrollbar = Scrollbar(self, orient=VERTICAL, command=self.canvas.yview, **self.scrollbar_style)
        scrollbar.grid(row=0, column=5, sticky=NS, rowspan=row_length+1)

        # Assign a vertical scrollbar to the canvas.
        self.canvas.configure(yscrollcommand=scrollbar.set)
        self.canvas.create_window((0, 0), window=self.frame, anchor=NW)

        # Initialise the current row (of the CSV being displayed) to 1.
        self.row_num = 1

        # Create a title label showing the Name of the files.
        Label(self.frame, text="Name", **self.secondary_label_style).grid(row=0,column=0, pady=15, sticky = EW, padx=10)

        # Create a title label showing the file Extensions.
        Label(self.frame, text="Extension", **self.secondary_label_style).grid(row=0,column=1, pady=15, sticky = EW)

        # Create a title label showing when the files were last modified.
        Label(self.frame, text="Modified", **self.secondary_label_style).grid(row=0,column=2, pady=15, sticky = EW)

        # Create a title label showing the length of the files.
        Label(self.frame, text="Length", **self.secondary_label_style).grid(row=0,column=3, pady=15, sticky = EW)

        # Create a label prompting the user to delete files.
        Label(self.frame, text="Delete", **self.secondary_label_style).grid(row=0,column=4, pady=15, sticky = EW, padx=20)

        self.button_list = []

        for x in range(0, len(self.file_info_list)):
            
            # Display the information about the CSV.
            self.display_row(self.file_info_list[x], x)

            
        # Update the scroll region of the canvas
        self.frame.update_idletasks()
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

        self.frame.pack(fill=BOTH, expand=TRUE)

        # Create the options frame
        options_frame = Frame(self, bg="white")
        options_frame.pack(side="bottom", fill="x")

        # Create an import button, allowing the user to import files.
        Button(options_frame, text="Import File", **self.button_style, command=self.import_csv).pack(expand=True, side="left",pady=5)

        # Create a Use Live Data button, navigating the user to the Connection screen.
        Button(options_frame, text="Use Live Data", **self.button_style, command = lambda:self.program_window.change_csv_frame("connect to sensor")).pack(expand=True, side="left",pady=5)

        

    def display_row(self, file, index):
        """
        Display a row of information about a CSV.
        """
            
        # Create a function for each button, allowing the user to select that CSV to display.
        change_function = self.create_csv_function(file["name"], file["extension"], index)

        # Display a button containing the name of the current CSV.
        button = Button(self.frame, text= file["name"] , **self.button_style, command=change_function)
        button.grid(row=self.row_num, column=0, sticky=NSEW, padx=20)
        self.button_list.append(button)

        # Display a label containing the file extension.
        Label(self.frame, text= file["extension"],**self.secondary_label_style).grid(row=self.row_num, column=1, sticky=NSEW, padx=20)

        # Display a label containing the time the file was last modified.
        Label(self.frame, text= file["modification_time"], **self.secondary_label_style).grid(row=self.row_num, column=2, sticky=NSEW, padx=20)

        # Display a label containing the length of the file.
        Label(self.frame, text= file["length"], **self.secondary_label_style).grid(row=self.row_num, column=3, sticky=NSEW, padx=20)

        # Create a function allowing the user to delete the current file.
        delete_function = self.create_delete_function(self.row_num, self.frame, file, self.canvas)

        # Display a button allowing the user to delete the current file.
        Button(self.frame, image=self.delete_image, command=delete_function, **self.image_style).grid(row=self.row_num, column=4, sticky=NSEW, padx=20)

        # Increment the current row of data to be displayed.
        self.row_num+=1

    
    def import_csv(self):
        """
        Allows the user to import a CSV to the project.
        Copies the file to the ./CSVs directory.
        """

        # Open the file dialog.
        file_path = askopenfilename()

        # Retrieve the file name from the path.
        file_name = os.path.basename(file_path)

        # Copy the chosen file to the ./CSVs directory.
        shutil.copy(file_path, self.directory_path)

        # Retrieve the information about the file.
        self.get_file_info(file_name)

        # Display the information to the screen.
        self.display_row(self.file_info_list[-1])



    def create_delete_function(self, row, frame, file, canvas):
        """
        Create a unique delete function for each row, when the delete button is pressed it deletes that row of data.
        """
        return lambda: self.delete_file(frame, row, file["name"], file["extension"], canvas)
    
    def create_csv_function(self, name, extension, index):
        """
        Create a unique function for each CSV, navigating to the Display screen with the chosen CSV as the argument.
        """

        # Append the file name to the file extension.
        file_name = name + extension

        # Create the file path.
        file_path = os.path.join(self.directory_path, file_name)

        # Navigate to the Display screen with the current CSV as an argument.
        return lambda:self.store_new_csv(file_path, index)
    
    def store_new_csv(self, file_path, index):



        # so only one button is green
        for x in self.button_list:
            x.config(bg = "#42c4ee")

        self.button_list[index].config(bg="green")
        self.simulation_frame.store_csv(file_path)

        # now tell waiting screen to turn its button blue

    def delete_file(self, frame, row, name, extension, canvas):
        """
        Delete the chosen file from the ./CSVs directory.
        Remove the row of data from the screen.
        """

        # Append the file name to the file extension.
        file_name = name + extension

        # Create the file path.
        file_path = os.path.join(self.directory_path, file_name)

        # Remove the file from the ./CSVs directory.
        os.remove(file_path)

        # For each widget on the frame,
        for child in frame.winfo_children():

            # Retrieve the information about the current widget.
            info = child.grid_info()

            # If the row of the current widget == the row of the data being deleted,
            if 'row' in info and info['row'] == row:

                # Remove the widget from the screen.
                child.grid_forget()

        # Adjust the scroll region of the canvas
        frame.update_idletasks()
        canvas.configure(scrollregion=canvas.bbox("all"))






            

                    


                