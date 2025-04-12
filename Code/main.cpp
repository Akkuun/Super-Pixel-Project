#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Progress.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_RGB_Image.H>
#include <fstream>
#include <vector>
#include <iostream>
#include <FL/Fl_Int_Input.H>
#include "Image.h"
#include <FL/Fl_Choice.H>
#include <cstdlib> // For system()

#include <iomanip>
#include <sstream>

using namespace std;

string selected_file;
bool genererSLICC = false;
bool contourSLICC = false;
bool compressSLICC = false;
bool genererMeanShift = false;
bool contourMeanShift = false;
bool compressMeanShift = false;
int k = 700;
int m = 55;
float spatial_radius = 10.0f;
float color_radius = 10.0f;
int max_iterations = 100;
Fl_Button *process_button;

vector<Fl_Box *> listeDesaImagesAfficher;
Fl_Box *image_box;
Fl_RGB_Image *displayed_image = nullptr; // Global variable to store the image
Fl_Check_Button *contour_slicc_button = nullptr;
Fl_Check_Button *compress_slicc_button = nullptr;
Fl_Check_Button *contour_mean_shift_button = nullptr;
Fl_Check_Button *compress_mean_shift_button = nullptr;
Fl_Button* save_as_button = nullptr;
Fl_Choice* format_choice = nullptr;
std::string selected_format = "JPG"; // Default format


void save_as_callback(Fl_Widget* w, void* data) {
    // Update the selected format based on the dropdown choice
    selected_format = ((Fl_Choice*)data)->text();
}

std::string floatToString(double value, int precision) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

void update_process_button() {
    bool slicc_ready = genererSLICC && !selected_file.empty();
    bool mean_shift_ready = genererMeanShift && !selected_file.empty();
    if (slicc_ready || mean_shift_ready) {
        process_button->activate();
    } else {
        process_button->deactivate();
    }
}


Fl_RGB_Image *loadPPM(const char *filename) {
    ifstream file(filename, ios::binary);
    if (!file) {
        cerr << "Erreur : Impossible d'ouvrir " << filename << endl;
        return nullptr;
    }

    string format;
    int width, height, maxVal;

    file >> format;
    if (format != "P6") {
        cerr << "Format non supporté : " << format << endl;
        return nullptr;
    }

    file >> width >> height >> maxVal;
    file.ignore(); // Sauter le caractère de nouvelle ligne

    if (maxVal != 255) {
        cerr << "Seuls les fichiers avec une plage de 255 sont supportés" << endl;
        return nullptr;
    }

    vector<unsigned char> pixels(width * height * 3);
    file.read(reinterpret_cast<char *>(pixels.data()), pixels.size());

    return new Fl_RGB_Image(pixels.data(), width, height, 3);
}

void choose_file(Fl_Widget *w, void *data) {
    Fl_File_Chooser chooser(".", "*.{ppm}", Fl_File_Chooser::SINGLE, "Choose an Image");
    chooser.show();
    while (chooser.shown()) {
        Fl::wait();
    }
    if (chooser.value() != nullptr) {
        selected_file = chooser.value();
        Fl_Box *box = (Fl_Box *) data;
        box->label(selected_file.c_str());
        update_process_button();
    }
}


void process_image(Fl_Widget *w, void *data) {
    if (selected_file.empty()) {
        cout << "No file selected!" << endl;
        return;
    }

    Image img(selected_file, Image::PPM);
    img.read();


    if (genererSLICC) {
        Image imgLAB = img.RGBtoLAB();
        //imgLAB.write(selected_file);
        int N = img.getSize();
        string nomFichierSortieSLICC =
                selected_file.substr(0, selected_file.find_last_of('.')) + "_SLICC_" + to_string(k) + "_" +
                to_string(m) + ".ppm";
        imgLAB.SLICC(k, m, N, contourSLICC, nomFichierSortieSLICC);
        Image imgOUT = imgLAB.LABtoRGB();
        imgOUT.write(nomFichierSortieSLICC);
        //img.genererCourbePSNR(imgLAB, img, 100, 2000, 10, 50, N);

        if (compressSLICC) {
            Image imgOUTLAB = imgOUT.RGBtoLAB();
            img.compressionPallette(imgOUT, nomFichierSortieSLICC);
        }
        if (!nomFichierSortieSLICC.empty() && (selected_format == "JPG" || selected_format == "PNG")) {
            std::string converted_file = nomFichierSortieSLICC.substr(0, nomFichierSortieSLICC.find_last_of('.')) + "." + selected_format;
            std::string command = "convert " + nomFichierSortieSLICC + " " + converted_file; // Requires ImageMagick
            system(command.c_str());
            cout << "Image saved as: " << converted_file << endl;
        }


    }

    if (genererMeanShift) {
        Image imgLAB = img.RGBtoLAB();
        string nomFichierSortieMean = selected_file.substr(0, selected_file.find_last_of('.')) + "_MeanShift_" +
                                      floatToString(color_radius, 2) + "_" + floatToString(spatial_radius, 2) + ".ppm";
        Image segmentedImg = imgLAB.MeanShiftSegmentation(spatial_radius, color_radius, max_iterations,
                                                          contourMeanShift, nomFichierSortieMean);
        Image resultImg = segmentedImg.LABtoRGB();
        resultImg.write(nomFichierSortieMean);

        if (compressMeanShift) {
            Image imgOUTLAB = resultImg.RGBtoLAB();
            img.compressionPallette(resultImg, nomFichierSortieMean);
        }



    }

    Fl_Progress *progress = (Fl_Progress *) data;
    progress->value(100);
}

void toggle_genererSLICC(Fl_Widget *w, void *data) {
    genererSLICC = !genererSLICC;
    Fl_Input **inputs = (Fl_Input **) data;
    for (int i = 0; i < 2; ++i) {
        if (genererSLICC) {
            inputs[i]->activate();
            contour_slicc_button->activate();
            compress_slicc_button->activate();
        } else {
            inputs[i]->deactivate();
            contour_slicc_button->deactivate();
            compress_slicc_button->deactivate();
        }
    }
    update_process_button();
}

void toggle_genererMeanShift(Fl_Widget *w, void *data) {
    genererMeanShift = !genererMeanShift;
    Fl_Input **inputs = (Fl_Input **) data;
    for (int i = 0; i < 2; ++i) {
        if (genererMeanShift) {
            inputs[i]->activate();
            contour_mean_shift_button->activate();
            compress_mean_shift_button->activate();
        } else {
            inputs[i]->deactivate();
            contour_mean_shift_button->deactivate();
            compress_mean_shift_button->deactivate();
        }
    }
    update_process_button();
}

int main(int argc, char **argv) {
    Fl_Window* window = new Fl_Window(800, 800, "Image Processing");

    Fl_Button* choose_button = new Fl_Button(10, 10, 100, 30, "Choose Image");
    Fl_Box* file_box = new Fl_Box(120, 10, 470, 30, "No file chosen");

    Fl_Check_Button* slicc_button = new Fl_Check_Button(10, 50, 150, 30, "Generate SLICC");
    contour_slicc_button = new Fl_Check_Button(10, 90, 150, 30, "Contour SLICC");
    compress_slicc_button = new Fl_Check_Button(10, 130, 150, 30, "Compress SLICC");
    Fl_Input* k_input = new Fl_Int_Input(300, 50, 100, 30, "k:");

    k_input->deactivate();
    Fl_Input* m_input = new Fl_Int_Input(300, 90, 100, 30, "m:");
    m_input->deactivate();

    Fl_Check_Button* mean_shift_button = new Fl_Check_Button(10, 170, 150, 30, "Generate Mean Shift");
    contour_mean_shift_button = new Fl_Check_Button(10, 210, 150, 30, "Contour Mean Shift");
    compress_mean_shift_button = new Fl_Check_Button(10, 250, 165, 30, "Compress Mean Shift");
    Fl_Input* spatial_input = new Fl_Int_Input(300, 170, 100, 30, "Spatial Radius:");
    spatial_input->deactivate();
    Fl_Input* color_input = new Fl_Int_Input(300, 210, 100, 30, "Color Radius:");
    color_input->deactivate();

    save_as_button = new Fl_Button(10, 290, 150, 30, "Save As");
    format_choice = new Fl_Choice(300, 290, 100, 30, "Format:");
    format_choice->add("JPG|PNG");
    format_choice->value(0); // Default to JPG
    format_choice->callback(save_as_callback, format_choice);

    process_button = new Fl_Button(10, 330, 100, 30, "Process Image");
    Fl_Progress* progress = new Fl_Progress(120, 330, 470, 30);
    progress->minimum(0);
    progress->maximum(100);

    Fl_Input* slicc_inputs[] = {k_input, m_input};
    Fl_Input* mean_shift_inputs[] = {spatial_input, color_input};

    choose_button->callback(choose_file, file_box);
    slicc_button->callback(toggle_genererSLICC, slicc_inputs);
    contour_slicc_button->deactivate();
    compress_slicc_button->deactivate();
    mean_shift_button->callback(toggle_genererMeanShift, mean_shift_inputs);
    contour_mean_shift_button->deactivate();
    compress_mean_shift_button->deactivate();
    process_button->callback(process_image, progress);
    process_button->deactivate();

    k_input->callback([](Fl_Widget* w, void* data) {
        k = atoi(((Fl_Input*)w)->value());
    });
    m_input->callback([](Fl_Widget* w, void* data) {
        m = atoi(((Fl_Input*)w)->value());
    });

    spatial_input->callback([](Fl_Widget* w, void* data) {
        spatial_radius = atof(((Fl_Input*)w)->value());
    });
    color_input->callback([](Fl_Widget* w, void* data) {
        color_radius = atof(((Fl_Input*)w)->value());
    });

    contour_slicc_button->callback([](Fl_Widget* w, void* data) {
        contourSLICC = ((Fl_Check_Button*)w)->value();
    });
    compress_slicc_button->callback([](Fl_Widget* w, void* data) {
        compressSLICC = ((Fl_Check_Button*)w)->value();
    });

    contour_mean_shift_button->callback([](Fl_Widget* w, void* data) {
        contourMeanShift = ((Fl_Check_Button*)w)->value();
    });
    compress_mean_shift_button->callback([](Fl_Widget* w, void* data) {
        compressMeanShift = ((Fl_Check_Button*)w)->value();
    });

    window->end();
    window->show(argc, argv);
    return Fl::run();
}