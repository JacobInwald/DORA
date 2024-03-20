'use client';

import { format } from 'date-fns';
import { Calendar as CalendarIcon, Link } from 'lucide-react';
import { cn } from '@/lib/utils';
import { Button } from '@/components/ui/button';
import { Calendar } from '@/components/ui/calendar';
import { Popover, PopoverContent, PopoverTrigger } from '@/components/ui/popover';
import { TimePickerDemo } from '@/components/time-picker/time-picker-demo';
import {
  Form,
  FormControl,
  FormDescription,
  FormField,
  FormItem,
  FormLabel,
  FormMessage,
} from '@/components/ui/form';
import { z } from 'zod';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { useToast } from '@/components/ui/use-toast';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from './ui/select';

const formSchema = z.object({
  dateTime: z.date(),
  repeat: z.enum(['daily', 'weekly', 'monthly', 'yearly', 'weekdays']).optional(),
});

type FormSchemaType = z.infer<typeof formSchema>;

export function Schedule() {
  const { toast } = useToast();

  const form = useForm<FormSchemaType>({
    resolver: zodResolver(formSchema),
  });

  function onSubmit(data: FormSchemaType) {
    toast({
      title: 'You submitted the following values:',
      description: (
        <pre>
          <code>{JSON.stringify(data, null, 2)}</code>
        </pre>
      ),
    });
  }

  return (
    <Form {...form}>
      <form className="w-full mx-auto flex flex-col gap-2" onSubmit={form.handleSubmit(onSubmit)}>
        <div>
          <FormLabel className="text-left">Schedule DORA</FormLabel>
          <FormDescription>Set the date and time for DORA to start tidying!</FormDescription>
        </div>
        <FormField
          control={form.control}
          name="dateTime"
          render={({ field }) => (
            <FormItem className="flex flex-col">
              <Popover>
                <FormControl>
                  <PopoverTrigger asChild>
                    <Button
                      variant="outline"
                      className={cn(
                        'w-full justify-start text-left font-normal',
                        !field.value && 'text-muted-foreground'
                      )}
                    >
                      <CalendarIcon className="mr-2 h-4 w-4" />
                      {field.value ? (
                        format(field.value, 'PPP HH:mm')
                      ) : (
                        <span>Pick a date and time</span>
                      )}
                    </Button>
                  </PopoverTrigger>
                </FormControl>
                <PopoverContent className="w-auto p-0">
                  <Calendar
                    mode="single"
                    selected={field.value}
                    onSelect={field.onChange}
                    initialFocus
                  />
                  <div className="p-3 border-t border-border">
                    <TimePickerDemo setDate={field.onChange} date={field.value} />
                  </div>
                </PopoverContent>
              </Popover>
            </FormItem>
          )}
        />
        <FormField
          control={form.control}
          name="repeat"
          render={({ field }) => (
            <FormItem>
              <Select
                onValueChange={field.onChange}
                defaultValue={field.value}
                disabled={!form.getValues().dateTime}
              >
                <FormControl>
                  <SelectTrigger>
                    <SelectValue placeholder="Doesn't repeat" />
                  </SelectTrigger>
                </FormControl>
                <SelectContent>
                  <SelectItem value="daily">
                    Daily at{' '}
                    {form.getValues().dateTime && format(form.getValues().dateTime, 'HH:mm')}
                  </SelectItem>
                  <SelectItem value="weekly">
                    Weekly on{' '}
                    {form.getValues().dateTime && format(form.getValues().dateTime, 'EEEE')}s
                  </SelectItem>
                  <SelectItem value="monthly">
                    Monthly on the{' '}
                    {form.getValues().dateTime && format(form.getValues().dateTime, 'do')}
                  </SelectItem>
                  <SelectItem value="yearly">
                    Annually on{' '}
                    {form.getValues().dateTime && format(form.getValues().dateTime, 'MMMM do')}
                  </SelectItem>
                  <SelectItem value="weekdays">
                    Weekdays (Mon-Fri) at{' '}
                    {form.getValues().dateTime && format(form.getValues().dateTime, 'HH:mm')}
                  </SelectItem>
                </SelectContent>
              </Select>
            </FormItem>
          )}
        />
        <Button className="w-full" type="submit">
          Schedule
        </Button>
        {form.watch('dateTime') ? (
          <p className="text-muted-foreground text-xs">
            DORA will start tidying on {format(form.watch('dateTime'), 'PPP')} at{' '}
            {format(form.watch('dateTime'), 'HH:mm')} and will repeat{' '}
            {form.watch('repeat') ? (
              <span>
                {
                  {
                    daily: `daily at ${format(form.watch('dateTime'), 'HH:mm')}`,
                    weekly: `weekly on  ${format(form.watch('dateTime'), 'EEEE')}`,
                    monthly: `monthly on ${format(form.watch('dateTime'), 'do')}`,
                    yearly: `annually on ${format(form.watch('dateTime'), 'MMMM do')}`,
                    weekdays: `every weekday at ${format(form.watch('dateTime'), 'HH:mm')}`,
                  }[form.watch('repeat') as 'daily' | 'weekly' | 'monthly' | 'yearly' | 'weekdays']
                }
              </span>
            ) : (
              'once'
            )}
            .
          </p>
        ) : null}
      </form>
    </Form>
  );
}
