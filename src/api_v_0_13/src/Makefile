INCLUDE_DIR=../inc
OBJS_DIR=../objs
LIB_DIR=../lib
RET=0

all: vehiclecontrol.o 
	ar rcs $(LIB_DIR)/libvehicle.a $(OBJS_DIR)/*.o
	ranlib $(LIB_DIR)/libvehicle.a;

vehiclecontrol.o: vehiclecontrol.c vehiclecontrol.h createdirs copyheader
	gcc -o $(OBJS_DIR)/vehiclecontrol.o -g -c -Wall vehiclecontrol.c -I $(INCLUDE_DIR) -L $(LIB_DIR)/
	
createdirs:
	mkdir -p $(INCLUDE_DIR)
	mkdir -p $(OBJS_DIR)
	mkdir -p $(LIB_DIR)

copyheader:
	cp -f vehiclecontrol.h $(INCLUDE_DIR)

cleanall: clean
	rm -rf $(INCLUDE_DIR)
	rm -rf $(OBJS_DIR)
	rm -rf $(LIB_DIR)

clean: clearobjs clearlib clearinclude
	rm -f ./*~ ./*.o

clearobjs:
	rm -f $(OBJS_DIR)/*

clearinclude:
	rm -f $(INCLUDE_DIR)/*

clearlib:
	rm -f $(LIB_DIR)/libvehicle.a


