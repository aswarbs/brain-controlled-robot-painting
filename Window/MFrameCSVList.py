import datetime
from Window import *
from tkinter import *
import os # to retrieve files
from tkinter.filedialog import askopenfilename
import shutil

class FrameCSVList(FrameBase):
    """
    A frame which lists the CSVs in the ./CSVs directory onto the screen.
    """

    def __init__(self, master:Tk, main_window, parent_windows, **args):


        self.parent_windows = parent_windows
        self.main_window = main_window
        
        super().__init__(master, main_window)

        self.pack_propagate(0)  # Disable automatic resizing
        
        self.configure(highlightthickness=1, highlightbackground="black")
        

        self.display_files()

    def get_file(self, path, name):
        """
        Retrieves the file information for a specified file.
        Retrieves the file name, file extension, modification time, and length.
        """
        
        file_stat = os.stat(path)
        new_file_name = name.split(".")[0]
        modification_time = datetime.datetime.fromtimestamp(file_stat.st_mtime).strftime("%d/%m/%y %H:%M:%S")
        file_extension = os.path.splitext(name)[1]
        file_length = file_stat.st_size
        file_info = {
            "name": new_file_name,
            "modification_time": modification_time,
            "extension": file_extension,
            "length": file_length
        }
        return file_info

    def get_file_info(self, file_name):
        """
        Retrieves the file information for a file.
        """

        file_path = os.path.join(self.directory_path, file_name)
        
        file_info = self.get_file(file_path, file_name)

        self.file_info_list.append(file_info)


            

    def display_files(self):

        self.delete_image = PhotoImage(file='./Assets/delete.png')

        self.directory_path = "./CSVs"

        self.file_info_list = []

        for file_name in os.listdir(self.directory_path):
            self.get_file_info(file_name)

        # set number of rows to number of csv files
        row_length = len(self.file_info_list)

        # set grid propagate to 0 to stop the frame from resizing
        self.grid_propagate(0)

        # set the 1st column to weight 1 to allow the canvas to fill the space
        self.columnconfigure(1,weight=1)
        self.rowconfigure(1, weight=1)

        self.canvas = Canvas(self, bg="white")
        self.canvas.grid(row=0, column=0, sticky=NSEW, columnspan=2, rowspan=row_length+1)

        # Create a frame inside the canvas to hold the buttons
        self.frame = Frame(self.canvas, bg="white")

        self.frame.columnconfigure(0,weight=1)
        

        # Create a scrollbar and associate it with the canvas
        scrollbar = Scrollbar(self, orient=VERTICAL, command=self.canvas.yview, **self.scrollbar_style)
        scrollbar.grid(row=0, column=5, sticky=NS, rowspan=row_length+1)

        self.canvas.configure(yscrollcommand=scrollbar.set)

        self.canvas.create_window((0, 0), window=self.frame, anchor=NW)

        self.row_num = 1

        Label(self.frame, text="Name", **self.secondary_label_style).grid(row=0,column=0, pady=15, sticky = EW, padx=10)

        Label(self.frame, text="Extension", **self.secondary_label_style).grid(row=0,column=1, pady=15, sticky = EW)

        Label(self.frame, text="Modified", **self.secondary_label_style).grid(row=0,column=2, pady=15, sticky = EW)

        Label(self.frame, text="Length", **self.secondary_label_style).grid(row=0,column=3, pady=15, sticky = EW)

        Label(self.frame, text="Delete", **self.secondary_label_style).grid(row=0,column=4, pady=15, sticky = EW, padx=20)


        for file in self.file_info_list:

            self.display_row(file)

            
        # Update the scroll region of the canvas
        self.frame.update_idletasks()
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

        self.frame.pack(fill=BOTH, expand=TRUE)

        # Create the options frame
        options_frame = Frame(self, bg="white")
        options_frame.pack(side="bottom", fill="x")

        Button(options_frame, text= "Back", command=lambda:self.parent_window.change_frame("main menu"), **self.button_style).pack(expand=True, side="left", pady=5)

        Button(options_frame, text="Import File", **self.button_style, command=self.import_csv).pack(expand=True, side="left",pady=5)

        Button(options_frame, text="Use Live Data", **self.button_style, command = lambda:self.parent_windows.change_csv_frame("connect to sensor")).pack(expand=True, side="left",pady=5)

        

    def display_row(self, file):
            
        change_function = self.create_csv_function(file["name"], file["extension"])
        Button(self.frame, text= file["name"] , **self.button_style, command=change_function).grid(row=self.row_num, column=0, sticky=NSEW, padx=20)

        Label(self.frame, text= file["extension"],**self.secondary_label_style).grid(row=self.row_num, column=1, sticky=NSEW, padx=20)

        Label(self.frame, text= file["modification_time"], **self.secondary_label_style).grid(row=self.row_num, column=2, sticky=NSEW, padx=20)

        Label(self.frame, text= file["length"], **self.secondary_label_style).grid(row=self.row_num, column=3, sticky=NSEW, padx=20)

        delete_function = self.create_delete_function(self.row_num, self.frame, file, self.canvas)
        Button(self.frame, image=self.delete_image, command=delete_function, **self.image_style).grid(row=self.row_num, column=4, sticky=NSEW, padx=20)

        self.row_num+=1

    
    def import_csv(self):

        # open file dialog

        file_path = askopenfilename()

        # add chosen file to list

        file_name = os.path.basename(file_path)

        print(file_path)
        print(file_name)

        shutil.copy(file_path, self.directory_path)

        self.get_file_info(file_name)
        self.display_row(self.file_info_list[-1])






        pass

    def use_live_data(self):
        pass

    def create_delete_function(self, row, frame, file, canvas):
        return lambda: self.delete_file(frame, row, file["name"], file["extension"], canvas)
    
    def create_csv_function(self, name, extension):
        file_name = name + extension
        file_path = os.path.join(self.directory_path, file_name)
        return lambda: self.parent_windows.change_csv_frame("csv display", path=file_path)

    def delete_file(self, frame, row, name, extension, canvas):

        # delete file from directory
        file_name = name + extension
        file_path = os.path.join(self.directory_path, file_name)
        os.remove(file_path)

        # Delete the widgets in the specified row
        for child in frame.winfo_children():
            info = child.grid_info()
            if 'row' in info and info['row'] == row:
                child.grid_forget()

        # Adjust the scroll region of the canvas
        frame.update_idletasks()
        canvas.configure(scrollregion=canvas.bbox("all"))






            

                    


                