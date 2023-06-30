from Window import *
from tkinter import *
from tkinter import colorchooser

class FrameFiltersWindow(FrameBase):
    """
    A frame to allow the user to apply filters to the current data.
    """

    def __init__(self, master:Tk, main_window):
        super().__init__(master, main_window)
        self.configure(highlightthickness=1, highlightbackground="black")
        self.pack_propagate(0)

        self.buttons = []
        self.colours = ["red", "yellow", "green", "blue", "purple"]
        self.remove_image = PhotoImage(file='./Assets/remove.png')

        self.filter_frame = Frame(self)
        self.filter_frame.config(bg="white")
        self.selected_item = "None"

        self.chosen_filters = []


        self.display_filters(self.filter_frame)

        self.display_chosen_filters(self.filter_frame)

        self.filter_frame.pack(fill=BOTH, expand=TRUE)

        self.display_colours_and_dropdown()

        

    def display_colours_and_dropdown(self):
        """
        Create a frame which displays the colour scheme options and the trajectory options.
        """

        # create a frame which will be on the top of the filters frame
        options_frame = Frame(self)
        options_frame.config(bg="white", highlightthickness=1, highlightbackground="black")

        # create a label to display the colour scheme
        Label(options_frame, text="Colour Scheme:", **self.secondary_label_style).pack(side=LEFT,padx=10, pady=10, expand=TRUE)

        # Create five square buttons displaying the current colour
        for i in range(5):
            button = Button(options_frame, width=1, height=1, bg=self.colours[i], command=lambda idx=i: self.change_color(idx), highlightthickness=1, highlightbackground="black")
            button.pack(side='left', padx=5, pady=10)
            self.buttons.append(button)

        
        # create 
        selected_var = StringVar(options_frame)
        selected_var.set("None")

        # Create a label and associate it with the StringVar
        label = Label(options_frame, text="Current Trajectory:", **self.secondary_label_style)
        label.pack(side=LEFT, padx=10, pady=10,expand=TRUE)


        dropdown = OptionMenu(options_frame, selected_var, "None", "Square", "Circle", "Face", command=self.handle_selection)
        dropdown.config(font=("Bahnschrift Light", 14), bg='white', fg='black', activebackground='gray', activeforeground='white', relief='flat')

        dropdown["menu"].config(font=("Bahnschrift Light", 14), bg='white', fg='black')
        dropdown.pack(side=LEFT, padx=10, pady=10, expand=TRUE)

        # pack the label to the top of the screen
        options_frame.pack(side=BOTTOM, fill=X)

    def display_filters(self, parent_frame):
        """
        Display a list of filters and allow the user to select filters to apply.
        """


        filters = ["Bandpass", "Common Spatial Pattern", "Artifact Removal", "Time Frequency Analysis"]

        self.row_num = 1
        self.button_list = []
        self.info_image = PhotoImage(file='./Assets/info.png')

        list_filters_frame = Frame(parent_frame)
        list_filters_frame.config(bg="white")


        Label(list_filters_frame, text="Choose Filters to Apply:", **self.secondary_label_style).pack(side=TOP, pady=10,)

        for name in filters:
            self.display_row(name, list_filters_frame)

        list_filters_frame.pack(side=LEFT, expand=TRUE, fill=Y)


    def display_chosen_filters(self, parent_frame):


        self.chosen_filters_frame = Frame(parent_frame)
        self.chosen_filters_frame.config(bg="white")

        Label(self.chosen_filters_frame, text="Chosen Filters:", **self.secondary_label_style).pack(side=TOP, pady=10)

        index=1
        for name in self.chosen_filters:
            self.display_chosen_filter(name, index, self.chosen_filters_frame)
            index+=1

        self.chosen_filters_frame.pack(side=RIGHT, fill=Y, expand=TRUE)

    def display_chosen_filter(self, name, index, parent_frame):
        """
        Display a row of information about the chosen filters.
        """

        current_label_frame = Frame(parent_frame)
        current_label_frame.config(bg="white")
        

        # Display a button containing the name of the current filter.
        label = Label(current_label_frame, text=str(index) + ". " + name, **self.secondary_label_style)
        label.pack(side=LEFT, pady=2)

        # Display a button allowing the user to get information about the current filter.
        Button(current_label_frame, image=self.remove_image, command=lambda:self.remove(index), **self.image_style).pack(side=LEFT, padx=2)


        current_label_frame.pack(side=TOP)

    def remove(self, index):
        del self.chosen_filters[index-1]

        self.chosen_filters_frame.destroy()

        self.display_chosen_filters(self.filter_frame)

        

    
    def display_row(self, name, parent_frame):
        """
        Display a row of information about a filter.
        """

        current_frame = Frame(parent_frame)
        current_frame.config(bg="white")


        # Display a button containing the name of the current filter.
        button = Button(current_frame, text=name, **self.button_style, command=lambda:self.add_to_filters(name))
        button.pack(side=LEFT, pady=2)
        self.button_list.append(button)



        
        # Display a button allowing the user to get information about the current filter.
        Button(current_frame, image=self.info_image, command=self.display_info(name), **self.image_style).pack(side=LEFT)

        current_frame.pack(side=TOP, padx=2)


    def add_to_filters(self, name):
        self.chosen_filters.append(name)

        self.display_chosen_filter(name, len(self.chosen_filters), self.chosen_filters_frame)
        pass

    def display_info(self, name):
        pass






        

            


    def handle_selection(self, selected_item):
        self.selected_item = selected_item




    def change_color(self,index):
        color = colorchooser.askcolor(title="Select Color")[1]
        if color:
            self.buttons[index].configure(bg=color)
            self.colours[index] = color

        print(self.colours)